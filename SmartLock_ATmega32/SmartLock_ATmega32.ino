/*
 * SmartLock_ATmega32.ino  -  MASTER firmware (ATmega32A, MightyCore, Arduino IDE)
 *
 * STATUS: THEORETICAL. Not compiled with avr-gcc and not run on real hardware.
 *         Logic was only checked on a PC with a simulation harness.
 *         Everything marked "HW-ASSUMPTION" must be verified on your board.
 *
 * Board settings (as in your Tools menu):
 *   Board: ATmega32 | Clock: External 14.7456 MHz | Bootloader: No bootloader
 *   EEPROM: retained | JTAG pins: disabled | Programmer: USBasp (slow)
 *   Upload with: Sketch > Upload Using Programmer
 *
 * Pin map (direct register access, so MightyCore pin-number mapping is NOT used):
 *   Keypad rows 1-4  : PA0..PA3 (DIP 40,39,38,37)  outputs
 *   Keypad cols 1-4  : PA4..PA7 (DIP 36,35,34,33)  inputs + pull-up
 *   LCD I2C SCL/SDA  : PC0 / PC1 (DIP 22 / 23)     hardware TWI (own driver, no Wire)
 *   UART0 RXD / TXD  : PD0 / PD1 (DIP 14 / 15)     to ESP32 (9600 8N1)
 *   Servo signal     : PD5 / OC1A (DIP 19)         Timer1 hardware PWM, 50 Hz
 *   Door reed switch : PB1 (DIP 2)                 optional, to GND, internal pull-up
 *   PB5/PB6/PB7 + RESET are reserved for ISP.
 *
 * Timers: Timer0 = millis() (Arduino core), Timer1 = servo PWM. No conflict.
 * UART0 is wired to the ESP32, so NO debug prints are sent on it.
 */

#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>

// ======================= USER CONFIGURATION =======================
#define USE_DOOR_SENSOR      0      // 0 = no reed switch (servo-state only), 1 = reed switch on PB1
#define LOCKED_ANGLE         20     // degrees, MUST be calibrated (HW-ASSUMPTION)
#define UNLOCKED_ANGLE       100    // degrees, MUST be calibrated (HW-ASSUMPTION)
#define UNLOCK_WINDOW_MS     10000UL
// 1 = after the 10 s window the servo returns to LOCKED_ANGLE (literal spec; with a
//     door sensor it waits until the door is closed). 0 = servo stays unlocked during setup.
#define RETURN_SERVO_TO_LOCKED_AFTER_WINDOW 1
#define SERVO_MIN_US         600    // pulse for 0 deg   (HW-ASSUMPTION, SG90 typical)
#define SERVO_MAX_US         2400   // pulse for 180 deg (HW-ASSUMPTION, SG90 typical)
#define SERVO_STEP_DEG       3      // slew step, softens current peaks
#define SERVO_STEP_MS        20
#define PIN_LEN              4
#define VOICE_MIN            3
#define VOICE_MAX            12
#define RX_MAX               32     // raw UART line limit (longer lines are rejected)
#define KEY_DEBOUNCE_MS      30
#define MSG_MS               1500
#define MAX_FAILED_ATTEMPTS  3      // 0 disables the temporary lockout
#define LOCKOUT_MS           30000UL
#define LCD_ADDR_FORCE       0      // 0 = auto-detect (0x27, 0x3F, ...), else e.g. 0x27

// ======================= STATE MACHINE =======================
enum State : uint8_t {
  DOOR_OPEN_SETUP,
  ENTER_NEW_PIN,
  CONFIRM_NEW_PIN,
  ENTER_NEW_VOICE_PASSWORD,
  CONFIRM_NEW_VOICE_PASSWORD,
  READY_TO_LOCK,
  DOOR_LOCKED,
  AUTHENTICATION_GRANTED,
  DOOR_UNLOCKED,
  ERROR_STATE
};

static State    state;
static char     pinIn[PIN_LEN];
static uint8_t  pinLen;
static char     newPin[PIN_LEN];
static char     newVoice[VOICE_MAX];
static uint8_t  newVoiceLen;
static uint32_t unlockStartMs;
static bool     relockPending;
static uint8_t  failedAttempts;
static bool     lockoutActive;
static uint32_t lockoutEndMs;
static const char* errMsgP;          // PROGMEM string

// temporary on-screen message
static bool     msgActive;
static uint32_t msgEndMs;

// ======================= SMALL HELPERS =======================
static void secureZero(void* p, size_t n) {
  volatile uint8_t* v = (volatile uint8_t*)p;
  while (n--) *v++ = 0;
}

// constant-time compare (length must already be equal)
static bool ctEq(const char* a, const char* b, uint8_t n) {
  uint8_t d = 0;
  for (uint8_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

static bool isDig(char c) { return c >= '0' && c <= '9'; }

static void buildNum(char* out, const char* pre, uint8_t n) {
  uint8_t i = 0;
  while (pre[i] && i < 12) { out[i] = pre[i]; i++; }
  if (n >= 10) out[i++] = (char)('0' + (n / 10) % 10);
  out[i++] = (char)('0' + n % 10);
  out[i++] = 's';
  out[i] = 0;
}

// ======================= TWI (I2C) DRIVER =======================
// Master write only, ~100 kHz, bounded waits (never hangs the main loop).
static bool twiWait() {
  uint16_t t = 60000;
  while (!(TWCR & _BV(TWINT))) { if (--t == 0) return false; }
  return true;
}
static void twiStop()  { TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO); }
static void twiReset() { TWCR = 0; TWCR = _BV(TWEN); }

static void twiInit() {
  TWSR = 0;
  TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
  TWCR = _BV(TWEN);
}

static bool twiStartW(uint8_t addr7) {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  if (!twiWait()) { twiReset(); return false; }
  uint8_t s = TWSR & 0xF8;
  if (s != 0x08 && s != 0x10) { twiStop(); return false; }
  TWDR = (uint8_t)(addr7 << 1);
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait()) { twiReset(); return false; }
  if ((TWSR & 0xF8) != 0x18) { twiStop(); return false; }
  return true;
}

static bool twiWriteByte(uint8_t b) {
  TWDR = b;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait()) { twiReset(); return false; }
  return (TWSR & 0xF8) == 0x28;
}

// ======================= LCD 16x2 (HD44780 via PCF8574 backpack) =======================
// HW-ASSUMPTION: common backpack wiring P0=RS P1=RW P2=EN P3=BL P4..P7=D4..D7.
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

static uint8_t lcdAddr = 0;
static bool    lcdOk = false;
static char    lcdShadow[2][16];     // what is currently on the screen (avoids flicker)

static bool lcdNibble(uint8_t nib, bool rs) {
  uint8_t b = (uint8_t)(((nib & 0x0F) << 4) | LCD_BL | (rs ? LCD_RS : 0));
  if (!twiStartW(lcdAddr)) return false;
  bool ok = twiWriteByte((uint8_t)(b | LCD_EN)) && twiWriteByte(b);
  twiStop();
  return ok;
}

static void lcdSend(uint8_t v, bool rs) {
  if (!lcdOk) return;
  if (!lcdNibble(v >> 4, rs) || !lcdNibble(v & 0x0F, rs)) return;
}

static bool lcdProbe(uint8_t a) {
  bool ok = twiStartW(a);
  if (ok) twiStop();
  return ok;
}

static void lcdBegin() {
  memset(lcdShadow, 1, sizeof(lcdShadow));   // force first full draw
  static const uint8_t cand[] = { 0x27, 0x3F, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20,
                                  0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E };
#if LCD_ADDR_FORCE
  lcdAddr = LCD_ADDR_FORCE;
  lcdOk = lcdProbe(lcdAddr);
#else
  lcdOk = false;
  for (uint8_t i = 0; i < sizeof(cand); i++) {
    if (lcdProbe(cand[i])) { lcdAddr = cand[i]; lcdOk = true; break; }
  }
#endif
  if (!lcdOk) return;
  delay(50);
  lcdNibble(0x3, false); delay(5);
  lcdNibble(0x3, false); delayMicroseconds(150);
  lcdNibble(0x3, false); delayMicroseconds(150);
  lcdNibble(0x2, false); delayMicroseconds(150);   // 4-bit mode
  lcdSend(0x28, false);   // 2 lines, 5x8 font
  lcdSend(0x08, false);   // display off
  lcdSend(0x01, false); delay(3);   // clear (only at init)
  lcdSend(0x06, false);   // entry mode
  lcdSend(0x0C, false);   // display on, no cursor
}

// Writes a row only if its content changed.
static void lcdRow(uint8_t row, const char* s) {
  char line[16];
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) line[i] = s[i];
  for (; i < 16; i++) line[i] = ' ';
  if (memcmp(line, lcdShadow[row], 16) == 0) return;
  memcpy(lcdShadow[row], line, 16);
  if (!lcdOk) return;
  lcdSend((uint8_t)(0x80 | (row ? 0x40 : 0x00)), false);
  for (i = 0; i < 16; i++) lcdSend((uint8_t)line[i], true);
}

static void lcdRowP(uint8_t row, const char* p) {
  char buf[17];
  uint8_t i = 0;
  char c;
  while (i < 16 && (c = (char)pgm_read_byte(p + i)) != 0) { buf[i] = c; i++; }
  buf[i] = 0;
  lcdRow(row, buf);
}

static void showMsg(const char* p0, const char* p1, uint16_t ms) {
  lcdRowP(0, p0);
  lcdRowP(1, p1);
  msgEndMs = millis() + ms;
  msgActive = true;
}

// ======================= SERVO (Timer1, Fast PWM mode 14, ICR1 = TOP) =======================
static int16_t  servoCur, servoTgt;
static uint32_t servoLastMs;

static uint16_t angleToTicks(int16_t a) {
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  uint32_t us = (uint32_t)SERVO_MIN_US + ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * (uint32_t)a) / 180UL;
  return (uint16_t)((us * (F_CPU / 8000UL)) / 1000UL);   // tick = 8 / F_CPU
}

static void servoBegin(int16_t angle) {
  servoCur = servoTgt = angle;
  DDRD |= _BV(PD5);
  ICR1  = (uint16_t)((F_CPU / 8UL / 50UL) - 1UL);        // 20 ms period
  OCR1A = angleToTicks(angle);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);          // prescaler 8
}

static void servoSet(int16_t angle) { servoTgt = angle; }

static void servoUpdate(uint32_t now) {
  if (servoCur == servoTgt) return;
  if ((now - servoLastMs) < SERVO_STEP_MS) return;
  servoLastMs = now;
  if (servoCur < servoTgt) { servoCur += SERVO_STEP_DEG; if (servoCur > servoTgt) servoCur = servoTgt; }
  else                     { servoCur -= SERVO_STEP_DEG; if (servoCur < servoTgt) servoCur = servoTgt; }
  OCR1A = angleToTicks(servoCur);
}

// ======================= DOOR SENSOR (optional reed switch on PB1) =======================
#if USE_DOOR_SENSOR
static bool     doorRaw, doorStable;
static uint32_t doorChangeMs;
static void doorInit() {
  DDRB  &= (uint8_t)~_BV(PB1);
  PORTB |= _BV(PB1);
  delay(5);
  doorRaw = doorStable = ((PINB & _BV(PB1)) == 0);    // closed contact pulls the pin LOW
}
static void doorPoll(uint32_t now) {
  bool raw = ((PINB & _BV(PB1)) == 0);
  if (raw != doorRaw) { doorRaw = raw; doorChangeMs = now; }
  else if ((now - doorChangeMs) >= 50) doorStable = raw;
}
static bool doorClosed() { return doorStable; }
#else
static void doorInit() {}
static void doorPoll(uint32_t) {}
static bool doorClosed() { return true; }   // unknown: no sensor installed
#endif

// ======================= KEYPAD 4x4 =======================
// HW-ASSUMPTION: keypad ribbon order R1 R2 R3 R4 C1 C2 C3 C4 -> PA0..PA7.
// If your membrane order differs, change only these two arrays (after a continuity test).
static const uint8_t ROW_BIT[4] = { 0, 1, 2, 3 };
static const uint8_t COL_BIT[4] = { 4, 5, 6, 7 };
static const char KEYMAP[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
static uint8_t rowMask, colMask;
static char     kLast;
static uint32_t kChangeMs;
static bool     kReported;

static void keypadInit() {
  rowMask = colMask = 0;
  for (uint8_t i = 0; i < 4; i++) { rowMask |= _BV(ROW_BIT[i]); colMask |= _BV(COL_BIT[i]); }
  DDRA  |= rowMask;
  DDRA  &= (uint8_t)~colMask;
  PORTA |= (uint8_t)(rowMask | colMask);    // rows idle high, columns pulled up
  kLast = 0; kReported = false;
}

static char keypadScanRaw() {
  char found = 0;
  for (uint8_t r = 0; r < 4 && !found; r++) {
    PORTA = (uint8_t)((PORTA | rowMask) & ~_BV(ROW_BIT[r]));
    delayMicroseconds(10);
    uint8_t v = PINA;
    for (uint8_t c = 0; c < 4; c++) {
      if (!(v & _BV(COL_BIT[c]))) { found = KEYMAP[r][c]; break; }
    }
  }
  PORTA |= rowMask;
  return found;
}

// Returns a key exactly once per press (debounced), else 0. Never blocks.
static char keypadPoll(uint32_t now) {
  char raw = keypadScanRaw();
  if (raw != kLast) { kLast = raw; kChangeMs = now; kReported = false; return 0; }
  if (raw && !kReported && (now - kChangeMs) >= KEY_DEBOUNCE_MS) { kReported = true; return raw; }
  return 0;
}

// ======================= UART RX (voice text from ESP32) =======================
static char    rxBuf[RX_MAX];
static uint8_t rxLen;
static bool    rxOverflow;
static bool    voiceEvt, voiceEvtBad;
static char    voiceEvtBuf[VOICE_MAX];
static uint8_t voiceEvtLen;

// Normalisation: uppercase, keep only A-Z/0-9 (drops spaces, dots, CR). Any byte >= 0x80
// (non-ASCII, e.g. Arabic letters) makes the message invalid.
static void finishLine() {
  voiceEvt = true; voiceEvtLen = 0; voiceEvtBad = false;
  if (rxOverflow) {
    voiceEvtBad = true;
  } else {
    for (uint8_t i = 0; i < rxLen; i++) {
      uint8_t c = (uint8_t)rxBuf[i];
      if (c >= 0x80) { voiceEvtBad = true; break; }
      if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 32);
      if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
        if (voiceEvtLen >= VOICE_MAX) { voiceEvtBad = true; break; }
        voiceEvtBuf[voiceEvtLen++] = (char)c;
      }
    }
    if (!voiceEvtBad && voiceEvtLen < VOICE_MIN) voiceEvtBad = true;
  }
  // neutral diagnostic back to the bridge (does NOT reveal whether the word was correct)
  Serial.print(voiceEvtBad ? "ERR\n" : "RX\n");
}

static void uartPoll() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n') {
      bool had = (rxLen > 0) || rxOverflow;
      if (had) finishLine();
      rxLen = 0; rxOverflow = false;
      secureZero(rxBuf, sizeof(rxBuf));
      if (had) break;                    // handle one message per loop pass
    } else if (c == '\r') {
      // ignored
    } else {
      if (rxLen < RX_MAX) rxBuf[rxLen++] = (char)c;
      else rxOverflow = true;
    }
  }
}

// ======================= EEPROM CREDENTIAL STORE (A/B slots) =======================
// Two slots. A write goes to the INACTIVE slot: magic cleared first, body written, CRC,
// magic written LAST, then read back and verified. A torn write leaves a slot invalid
// and the previous slot stays active, so a half-written credential is never valid.
#define REC_MAGIC 0xA5
struct CredRecord {
  uint8_t magic;
  uint8_t seq;
  uint8_t valid;            // 1 = locked-cycle credentials valid, 0 = invalidated
  char    pin[PIN_LEN];
  uint8_t vlen;
  char    voice[VOICE_MAX];
  uint8_t crc;
};
#define SLOT_SIZE 32
static_assert(sizeof(CredRecord) <= SLOT_SIZE, "CredRecord too big for slot");

static CredRecord credRec;           // RAM copy of the active record
static uint8_t    activeSlot = 255;  // 255 = none

static uint8_t crc8(const uint8_t* d, uint8_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}
static uint8_t recCrc(const CredRecord& r) {
  return crc8(((const uint8_t*)&r) + 1, (uint8_t)(sizeof(CredRecord) - 2));
}
/*
static bool recValid(const CredRecord& r) {
  if (r.magic != REC_MAGIC) return false;
  if (r.valid > 1 || r.vlen > VOICE_MAX) return false;
  if (r.crc != recCrc(r)) return false;
  if (r.valid == 1) {
    if (r.vlen < VOICE_MIN) return false;
    for (uint8_t i = 0; i < PIN_LEN; i++) if (!isDig(r.pin[i])) return false;
  }
  return true;
}
*/
static void slotRead(uint8_t s, CredRecord& r) {
  eeprom_read_block(&r, (const void*)(uint16_t)(s * SLOT_SIZE), sizeof(r));
}
/*
static bool slotWrite(uint8_t s, const CredRecord& in) {
  CredRecord r = in;
  r.magic = REC_MAGIC;
  r.crc = recCrc(r);
  const uint8_t* p = (const uint8_t*)&r;
  uint16_t base = (uint16_t)(s * SLOT_SIZE);
  eeprom_update_byte((uint8_t*)base, 0x00);                      // 1) invalidate slot
  for (uint8_t i = 1; i < sizeof(r); i++)
    eeprom_update_byte((uint8_t*)(base + i), p[i]);              // 2) body + crc
  eeprom_update_byte((uint8_t*)base, REC_MAGIC);                 // 3) commit marker last
  CredRecord chk;
  slotRead(s, chk);
  return recValid(chk) && memcmp(&chk, &r, sizeof(r)) == 0;      // 4) verify
}
*/
static void loadCreds() {
  CredRecord a, b;
  slotRead(0, a); slotRead(1, b);
  bool va = recValid(a), vb = recValid(b);
  secureZero(&credRec, sizeof(credRec));
  activeSlot = 255;
  if (va && vb) activeSlot = ((int8_t)(a.seq - b.seq) > 0) ? 0 : 1;
  else if (va)  activeSlot = 0;
  else if (vb)  activeSlot = 1;
  if (activeSlot == 0) credRec = a;
  if (activeSlot == 1) credRec = b;
  secureZero(&a, sizeof(a)); secureZero(&b, sizeof(b));
}

static bool commitRecord(uint8_t valid, const char* pin, const char* voice, uint8_t vlen) {
  CredRecord r;
  secureZero(&r, sizeof(r));
  r.seq = (activeSlot == 255) ? 1 : (uint8_t)(credRec.seq + 1);
  r.valid = valid;
  if (valid) {
    memcpy(r.pin, pin, PIN_LEN);
    memcpy(r.voice, voice, vlen);
    r.vlen = vlen;
  }
  uint8_t target = (activeSlot == 0) ? 1 : 0;
  if (!slotWrite(target, r)) return false;
  slotRead(target, credRec);
  activeSlot = target;
  secureZero(&r, sizeof(r));
  return true;
}

static bool credsActive() { return activeSlot != 255 && credRec.valid == 1; }

// ======================= STATE LOGIC =======================
static void enterError(const char* pmsg) { errMsgP = pmsg; state = ERROR_STATE; }

static void doDenied() {
  pinLen = 0;
  failedAttempts++;
  if (MAX_FAILED_ATTEMPTS > 0 && failedAttempts >= MAX_FAILED_ATTEMPTS) {
    lockoutActive = true;
    lockoutEndMs = millis() + LOCKOUT_MS;
    failedAttempts = 0;
  }
  showMsg(PSTR("ACCESS DENIED"), PSTR(""), MSG_MS);
}

// Successful authentication (PIN or voice). The credentials are invalidated FIRST;
// if that cannot be stored and verified we do NOT unlock (fail secure).
static void doGrant() {
  if (!commitRecord(0, NULL, NULL, 0)) {
    pinLen = 0;
    showMsg(PSTR("SYSTEM ERROR"), PSTR("TRY AGAIN"), MSG_MS);
    return;
  }
  pinLen = 0; failedAttempts = 0; lockoutActive = false;
  state = AUTHENTICATION_GRANTED;
}

static void tryLock() {
#if USE_DOOR_SENSOR
  if (!doorClosed()) { showMsg(PSTR("DOOR IS OPEN"), PSTR("CLOSE DOOR FIRST"), MSG_MS); return; }
#endif
  if (!commitRecord(1, newPin, newVoice, newVoiceLen)) { enterError(PSTR("EEPROM WRITE ERR")); return; }
  secureZero(newPin, sizeof(newPin));
  secureZero(newVoice, sizeof(newVoice));
  newVoiceLen = 0; pinLen = 0;
  failedAttempts = 0; lockoutActive = false; relockPending = false;
  servoSet(LOCKED_ANGLE);
  state = DOOR_LOCKED;
}

static void onKey(char k) {
  switch (state) {
    case ENTER_NEW_PIN:
      if (isDig(k)) { if (pinLen < PIN_LEN) pinIn[pinLen++] = k; }
      else if (k == '*') pinLen = 0;
      else if (k == '#') {
        if (pinLen == PIN_LEN) { memcpy(newPin, pinIn, PIN_LEN); pinLen = 0; state = CONFIRM_NEW_PIN; }
        else showMsg(PSTR("PIN INCOMPLETE"), PSTR("NEED 4 DIGITS"), MSG_MS);
      }
      break;

    case CONFIRM_NEW_PIN:
      if (isDig(k)) { if (pinLen < PIN_LEN) pinIn[pinLen++] = k; }
      else if (k == '*') { pinLen = 0; state = ENTER_NEW_PIN; }
      else if (k == '#') {
        if (pinLen != PIN_LEN) showMsg(PSTR("PIN INCOMPLETE"), PSTR("NEED 4 DIGITS"), MSG_MS);
        else if (ctEq(pinIn, newPin, PIN_LEN)) { pinLen = 0; state = ENTER_NEW_VOICE_PASSWORD; }
        else { pinLen = 0; secureZero(newPin, sizeof(newPin)); state = ENTER_NEW_PIN;
               showMsg(PSTR("PIN MISMATCH"), PSTR("START OVER"), MSG_MS); }
      }
      break;

    case ENTER_NEW_VOICE_PASSWORD:
    case CONFIRM_NEW_VOICE_PASSWORD:
      if (k == '*') state = DOOR_OPEN_SETUP;       // restart whole setup
      break;

    case READY_TO_LOCK:
      if (k == '#') tryLock();
      else if (k == '*') state = DOOR_OPEN_SETUP;  // redo credentials
      break;

    case DOOR_LOCKED:
      if (lockoutActive) break;
      if (isDig(k)) { if (pinLen < PIN_LEN) pinIn[pinLen++] = k; }
      else if (k == '*') pinLen = 0;
      else if (k == '#') {
        if (pinLen != PIN_LEN) { pinLen = 0; showMsg(PSTR("PIN INCOMPLETE"), PSTR("NEED 4 DIGITS"), MSG_MS); }
        else if (credsActive() && ctEq(pinIn, credRec.pin, PIN_LEN)) doGrant();
        else doDenied();
        secureZero(pinIn, sizeof(pinIn));
      }
      break;

    case ERROR_STATE:
      if (k == '*') state = DOOR_OPEN_SETUP;
      break;

    default:
      break;   // GRANTED / UNLOCKED / OPEN_SETUP: keys ignored
  }
}

static void onVoice(bool bad) {
  switch (state) {
    case ENTER_NEW_VOICE_PASSWORD:
      if (bad) showMsg(PSTR("BAD WORD"), PSTR("3-12 LETTERS/NUM"), MSG_MS);
      else { memcpy(newVoice, voiceEvtBuf, voiceEvtLen); newVoiceLen = voiceEvtLen; state = CONFIRM_NEW_VOICE_PASSWORD; }
      break;

    case CONFIRM_NEW_VOICE_PASSWORD:
      if (bad) showMsg(PSTR("BAD WORD"), PSTR("3-12 LETTERS/NUM"), MSG_MS);
      else if (voiceEvtLen == newVoiceLen && ctEq(voiceEvtBuf, newVoice, newVoiceLen)) state = READY_TO_LOCK;
      else { secureZero(newVoice, sizeof(newVoice)); newVoiceLen = 0; state = ENTER_NEW_VOICE_PASSWORD;
             showMsg(PSTR("WORD MISMATCH"), PSTR("START OVER"), MSG_MS); }
      break;

    case DOOR_LOCKED:
      if (lockoutActive) break;
      if (bad) { showMsg(PSTR("BAD MESSAGE"), PSTR("IGNORED"), MSG_MS); break; }
      if (credsActive() && voiceEvtLen == credRec.vlen && ctEq(voiceEvtBuf, credRec.voice, voiceEvtLen)) doGrant();
      else doDenied();
      break;

    default:
      break;   // voice text is ignored in every other state
  }
}

static void stateTick(uint32_t now) {
  switch (state) {
    case DOOR_OPEN_SETUP:
      secureZero(newPin, sizeof(newPin));
      secureZero(newVoice, sizeof(newVoice));
      secureZero(pinIn, sizeof(pinIn));
      newVoiceLen = 0; pinLen = 0;
      state = ENTER_NEW_PIN;
      break;

    case AUTHENTICATION_GRANTED:
      servoSet(UNLOCKED_ANGLE);
      unlockStartMs = now;
      state = DOOR_UNLOCKED;
      break;

    case DOOR_UNLOCKED:
      if ((now - unlockStartMs) >= UNLOCK_WINDOW_MS) {
        relockPending = (RETURN_SERVO_TO_LOCKED_AFTER_WINDOW != 0);
        state = DOOR_OPEN_SETUP;       // old credentials are already invalid
      }
      break;

    default:
      break;
  }
  // servo returns to locked only in setup states (and, with a sensor, only if the door is closed)
  if (relockPending && state >= ENTER_NEW_PIN && state <= READY_TO_LOCK && doorClosed()) {
    servoSet(LOCKED_ANGLE);
    relockPending = false;
  }
}

// ======================= LCD RENDERING (idempotent, no flicker) =======================
static void renderPinRow(uint8_t row) {
  char b[17] = "PIN:";
  for (uint8_t i = 0; i < pinLen && i < PIN_LEN; i++) b[4 + i] = '*';
  b[4 + pinLen] = 0;
  lcdRow(row, b);
}

static void render(uint32_t now) {
  char b[17];
  switch (state) {
    case ENTER_NEW_PIN:
      lcdRowP(0, PSTR("NEW PIN (4 dig)")); renderPinRow(1); break;
    case CONFIRM_NEW_PIN:
      lcdRowP(0, PSTR("CONFIRM PIN")); renderPinRow(1); break;
    case ENTER_NEW_VOICE_PASSWORD:
      lcdRowP(0, PSTR("SAY VOICE WORD")); lcdRowP(1, PSTR("(send via phone)")); break;
    case CONFIRM_NEW_VOICE_PASSWORD:
      lcdRowP(0, PSTR("REPEAT THE WORD")); lcdRowP(1, PSTR("(send via phone)")); break;
    case READY_TO_LOCK:
      lcdRowP(0, PSTR("READY TO LOCK")); lcdRowP(1, PSTR("#=LOCK  *=REDO")); break;
    case DOOR_LOCKED:
      if (lockoutActive) {
        uint32_t left = (lockoutEndMs - now);
        buildNum(b, "WAIT ", (uint8_t)((left + 999UL) / 1000UL));
        lcdRowP(0, PSTR("TOO MANY TRIES")); lcdRow(1, b);
      } else {
        lcdRowP(0, PSTR("LOCKED:PIN/VOICE")); renderPinRow(1);
      }
      break;
    case AUTHENTICATION_GRANTED:
    case DOOR_UNLOCKED: {
      uint32_t el = now - unlockStartMs;
      uint32_t left = (el >= UNLOCK_WINDOW_MS) ? 0 : (UNLOCK_WINDOW_MS - el);
      buildNum(b, "RELOCK IN ", (uint8_t)((left + 999UL) / 1000UL));
      lcdRowP(0, PSTR("ACCESS GRANTED")); lcdRow(1, b);
      break;
    }
    case ERROR_STATE:
      lcdRowP(0, PSTR("SYSTEM ERROR")); lcdRowP(1, errMsgP); break;
    default: break;
  }
}

// ======================= ARDUINO ENTRY POINTS =======================
void setup() {
  Serial.begin(9600);
  rxLen = 0; rxOverflow = false; voiceEvt = false;
  pinLen = 0; newVoiceLen = 0; msgActive = false;
  failedAttempts = 0; lockoutActive = false; relockPending = false;
  secureZero(newPin, sizeof(newPin)); secureZero(newVoice, sizeof(newVoice));

  keypadInit();
  doorInit();
  twiInit();
  lcdBegin();

  // boot splash (blocking is fine here, setup only)
  lcdRowP(0, PSTR("SMART DOOR LOCK"));
  if (lcdOk) {
    char b[17] = "LCD @0x00";
    const char* hx = "0123456789ABCDEF";
    b[7] = hx[(lcdAddr >> 4) & 0x0F]; b[8] = hx[lcdAddr & 0x0F];
    lcdRow(1, b);
  } else {
    lcdRowP(1, PSTR("LCD NOT FOUND"));
  }
  delay(1000);

  loadCreds();
  if (credsActive()) {
    servoBegin(LOCKED_ANGLE);        // power came back while locked -> stay locked
    state = DOOR_LOCKED;
  } else {
    servoBegin(UNLOCKED_ANGLE);      // no valid credentials -> never trap the user
    state = DOOR_OPEN_SETUP;
  }
}

void loop() {
  uint32_t now = millis();
  servoUpdate(now);
  doorPoll(now);
  uartPoll();
  char k = keypadPoll(now);

  if (msgActive && (int32_t)(now - msgEndMs) >= 0) msgActive = false;
  if (lockoutActive && (int32_t)(now - lockoutEndMs) >= 0) lockoutActive = false;

  if (voiceEvt) {
    voiceEvt = false;
    if (!msgActive) onVoice(voiceEvtBad);
    secureZero(voiceEvtBuf, sizeof(voiceEvtBuf));
    voiceEvtLen = 0;
  }
  if (k && !msgActive) onKey(k);

  stateTick(now);
  if (!msgActive) render(now);
}
