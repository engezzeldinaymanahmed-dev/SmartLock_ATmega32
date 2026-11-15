# 🔐 Dual-Factor Smart Door Lock System (ATmega32A)

An embedded dual-factor authentication smart door lock built around the **ATmega32A** microcontroller (`14.7456 MHz`). The system combines **4×4 Keypad PIN entry** and **Acoustic Clap Pattern Detection** using an **FC-04 Sound Sensor**, with persistent one-time credential storage in internal **EEPROM**, real-time status feedback on a **16×2 I2C LCD**, and mechanical lock actuation via an **SG90 Servo Motor**.

---

## ✨ Key Features

- **Dual-Factor Authentication:** Supports unlocking via either a 4-digit numeric PIN or a customizable acoustic clap sequence (2–10 claps).
- **One-Time Dynamic Credentials:** Credentials are automatically invalidated upon successful unlock, requiring a fresh PIN and clap code setup for every new lock cycle.
- **Torn-Write Safe EEPROM Storage:** Dual-slot (A/B) circular EEPROM storage with sequence numbering and **CRC-8** integrity verification to prevent data corruption during power loss.
- **Brute-Force Protection:** Automatically triggers a 30-second temporary lockout after 3 consecutive failed authentication attempts.
- **Bare-Metal Register Optimization:** Direct AVR register manipulation for Timer1 50 Hz hardware PWM (smooth servo slew-rate control), TWI (I2C) driver without external library overhead, and debounced matrix keypad scanning.
- **Auto-Relock Mechanism:** Unlocks for a configurable 10-second window before automatically re-engaging the lock and entering setup mode.

---

## 🛠️ Hardware Components

| Component | Specification / Model | Interface / Pin (DIP-40) |
| :--- | :--- | :--- |
| **Microcontroller** | ATmega32A (External `14.7456 MHz` Crystal + 2× `22pF`) | Core MCU |
| **Acoustic Sensor** | FC-04 / LM393 Sound Detection Module | `PB0` (Pin 1) — Digital Out |
| **Keypad** | 4×4 Matrix Membrane Keypad | `PA0–PA7` (Pins 40–33) |
| **Display** | 16×2 Character LCD with PCF8574 I2C Backpack | `PC0` SCL (Pin 22), `PC1` SDA (Pin 23) |
| **Actuator** | TowerPro SG90 Micro Servo Motor (`50 Hz` PWM) | `PD5 / OC1A` (Pin 19) |
| **Programmer** | USBasp AVR ISP Programmer | `PB5–PB7` (MOSI, MISO, SCK) + `RESET` |

---

## ⚙️ Fuse Bits & IDE Configuration (MightyCore)

To ensure accurate PWM timing and disable JTAG interference on `PORTC` (`PC2–PC5`), configure the board in Arduino IDE as follows and run **Burn Bootloader** once via USBasp:

- **Board:** `ATmega32`
- **Clock:** `External 14.7456 MHz`
- **Bootloader:** `No bootloader`
- **EEPROM:** `EEPROM retained`
- **JTAG pins:** `JTAG disabled`
- **Programmer:** `USBasp (slow)`

---

## 🔄 System State Machine Workflow

1. **Setup Phase (`DOOR_OPEN_SETUP`):**
   - Enter and confirm a new **4-digit PIN** (`#` to confirm, `*` to clear).
   - Clap **2 to 10 times** (`1.5s` silence timeout registers the count) and repeat to confirm.
   - Press `#` at `READY TO LOCK` to save credentials to EEPROM and rotate the servo to `20°` (Locked).
2. **Locked Phase (`DOOR_LOCKED`):**
   - Enter the saved 4-digit PIN + `#` **OR** clap the exact saved number of times.
   - 3 wrong attempts trigger a `30s` cooldown (`TOO MANY TRIES`).
3. **Unlocked Phase (`DOOR_UNLOCKED`):**
   - Servo moves smoothly to `100°` (Unlocked) for `10s` (`RELOCK IN 10s`), then resets for the next cycle.

---

## 👨‍💻 Authors

- **Ezzeldin Ayman**
- **Youssef Ahmed**

*Microprocessor Course Project (`EEC 214`) — Higher Technological Institute (HTI), 10th of Ramadan City.*
