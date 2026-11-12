
## Prerequisites
- ESP32 board (any variant with ≥4MB flash)
- BH1750 light sensor wired to **SDA=GPIO21**, **SCL=GPIO22** (I2C address `0x23`)
- Relay on **GPIO25**, alarm on **GPIO26**
- BOOT button on **GPIO0** (built-in on most boards)
- Arduino IDE with **ESP32 core ≥2.0** (3.x supported)

## Required Libraries (install via Library Manager)
- **BH1750** (by Christopher Laws)
- **Wire**, **WiFi**, **WebServer**, **DNSServer**, **Preferences** — bundled with ESP32 core

## Build & Flash
1. Open the modified `.ino` file in Arduino IDE.
2. Select your ESP32 board and correct COM port.
3. Click **Upload**.
4. On first boot (or after factory reset), the device creates a WiFi AP:
   - **SSID:** `ESP32_BH1750`
   - **Password:** `12345678`

## First Connection
1. On phone/PC, connect to `ESP32_BH1750`.
2. Captive portal opens automatically; if not, browse to **`http://192.168.4.1`**.
3. Dashboard shows live lux, relay, alarm, uptime, sensor status.

## Configure
- **Settings tab** → set `luxLow` / `luxHigh` (relay thresholds), `alarmLux` / `alarmClear` (alarm thresholds), enable/disable relay & alarm, invert relay. Tap **Save Settings**.
- **WiFi / AP tab** → change SSID / password. Tap **Apply & Reboot**. Device restarts in ~1.5s; reconnect with new credentials.

## Factory Reset
- **Option A:** Hold **BOOT (GPIO0)** for **5 seconds** → erases config, restarts.
- **Option B:** **Reset tab** → **Confirm Factory Reset**.

## Behavior Notes
- Relay turns **ON** when lux drops **below `luxLow`**, **OFF** when lux reaches **`luxHigh`** (hysteresis).
- Alarm turns **ON** above **`alarmLux`**, **OFF** below **`alarmClear`**.
- **Manual override** (Dashboard → Toggle Relay) lasts 60 s, then auto logic resumes.
- **Sensor loss:** after 10 s stale → 60 s more → relay forced OFF, alarm ON (fail-safe).
- **Config save:** throttled (15 s debounce, 120 s minimum gap) to protect NVS.

## Troubleshooting
- **AP not appearing:** check power; hold BOOT 5s to factory-reset, then power-cycle.
- **Sensor shows ERR:** verify BH1750 wiring, 3.3V power, pull-ups (most modules have them).
- **Relay never toggles:** confirm `relayEnabled=1` and `invertRelay` matches your relay module's logic.
- **Can't reach `192.168.4.1`:** disable mobile data; ensure you're on the AP network; try `http://192.168.4.1` explicitly (not HTTPS).
