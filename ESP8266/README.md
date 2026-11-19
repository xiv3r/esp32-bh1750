### 1. Hardware Pinout & Wiring

| Peripheral | Module Pin | ESP8266 GPIO | NodeMCU / D1 Mini Pin | Details |
| :--- | :--- | :--- | :--- | :--- |
| **BH1750 Sensor** | **VCC** | 3.3V | `3V3` | 3.3V DC power |
| | **GND** | GND | `GND` | Common ground |
| | **SDA** | GPIO 5 | **`D1`** | I2C Data line |
| | **SCL** | GPIO 4 | **`D2`** | I2C Clock line |
| | **ADDR** | GND | `GND` | Sets default I2C address to `0x23` |
| **Relay Module** | **VCC** | 5V (VIN) / 3.3V | `VIN` or `3V3` | Check your relay board rating |
| | **GND** | GND | `GND` | Common ground |
| | **IN** | GPIO 14 | **`D5`** | Relay control output |
| **Factory Reset** | **Button** | GPIO 0 | **`D3`** / `FLASH` | Built-in onboard FLASH button |

---

### 2. Flashing the Firmware

1. **Prerequisites in Arduino IDE**:
   - Ensure the ESP8266 board package is installed: `Tools` → `Board` → `Boards Manager...` → Search `esp8266` (version 3.1.2+).
   - Ensure the BH1750 library is installed: `Sketch` → `Include Library` → `Manage Libraries...` → Search **`BH1750`** (by Christopher Laws).
2. **Board Settings**:
   - **Board**: `NodeMCU 1.0 (ESP-12E Module)` or `Generic ESP8266 Module`
   - **Flash Size**: `4MB (FS:2MB)` or `4MB (FS:1MB)`
   - **Upload Speed**: `115200` or `921600`
3. **Upload**:
   - Open [`esp8266_bh1750_sketch.ino`](esp8266_bh1750_controller/esp8266_bh1750_controller.ino).
   - Connect your ESP8266 via USB and hit **Upload**.

> **Note**: In line with the production specification, the controller runs in **zero-serial mode** (no UART output) for maximum efficiency and security. All telemetry is served directly through the web dashboard.

---

### 3. Connecting to the Controller

1. **Join the Wi-Fi Network**:
   - Scan for Wi-Fi networks on your phone, tablet, or PC.
   - **SSID**: `ESP8266_BH1750`
   - **Password**: `12345678`
2. **Access the Dashboard**:
   - **Captive Portal**: Most mobile devices will immediately pop up a "Sign in to network" prompt—tap it to open the dashboard.
   - **Browser**: Or open any browser and go to `http://192.168.4.1`.

---

### 4. Control & Configuration Features

- **Live Dashboard**: Real-time lux dial gauge, ambient classification (Darkness, Moonlight, Indoor, Daylight, Direct Sun), relay status, and 24-hour min/max/average statistics.
- **Relay Automation Modes**:
  - `Above High`: Turn ON when light exceeds threshold; turn OFF when light drops below (High − Hysteresis).
  - `Below Low`: Turn ON when dark (e.g., night lights); turn OFF when light exceeds (Low + Hysteresis).
  - `Window Inside / Outside`: Dual-threshold band triggers.
- **Relay Protection**: Configurable anti-chatter dwell times (minimum ON / OFF duration) and invert logic (Active HIGH vs. Active LOW).
- **AP Configuration**: Change the SSID, password, channel, or hide the network from the UI.

---

### 5. Factory Reset

- **Hardware Button (Locked-Out Recovery)**:
  - Press and hold the onboard **FLASH button (GPIO 0 / D3)** for **5 continuous seconds**.
  - The onboard LED will flash rapidly and the system will restore all factory defaults and reboot.
- **Web UI**: Go to the **System Diagnostics** tab and click **"Factory Reset"**.
