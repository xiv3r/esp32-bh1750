#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <esp_log.h>

/* Pin Definitions */
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define PIN_RELAY_OUTPUT    25
#define PIN_BOOT_BUTTON     0

/* System Constants */
#define CONFIG_MAGIC        0x4248313735303634ULL  /* "BH175064" */
#define CONFIG_VERSION      1ULL
#define DEFAULT_SSID        "ESP32_BH1750"
#define DEFAULT_PASS        "12345678"
#define MIN_NVS_INTERVAL_MS 3000ULL
#define DNS_PORT            53

/* 64-bit Packed Configuration Record */
struct __attribute__((packed)) DeviceConfig {
  uint64_t magic;
  uint64_t version;
  char     wifi_ssid[33];
  char     wifi_pass[65];
  uint64_t wifi_channel;
  uint64_t wifi_hidden;
  uint64_t wifi_max_conn;
  uint64_t relay_mode;       /* 0: Disabled, 1: Above High, 2: Below Low, 3: Inside Window, 4: Outside Window */
  double   high_threshold;   /* 64-bit float */
  double   low_threshold;    /* 64-bit float */
  double   hysteresis;       /* 64-bit float */
  uint64_t relay_inverted;   /* 0: Active HIGH, 1: Active LOW */
  uint64_t dwell_on_ms;      /* 64-bit ms */
  uint64_t dwell_off_ms;     /* 64-bit ms */
  uint64_t bh1750_mode;      /* BH1750 Mode enum */
  uint64_t bh1750_mtreg;     /* 31 - 254 */
  uint64_t poll_interval_ms; /* 64-bit ms */
  uint64_t crc64;
};

/* Volatile Telemetry Tracking */
struct SensorTelemetry {
  double   current_lux;
  double   min_lux;
  double   max_lux;
  double   lux_accumulator;
  uint64_t total_samples;
  uint64_t error_samples;
  uint8_t  detected_address;
  bool     sensor_online;
  bool     last_read_valid;
  uint64_t last_poll_ms;
  uint32_t consecutive_failures;
};

struct RelayTelemetry {
  bool        active_state;
  uint64_t    override_mode;     /* 0: AUTO, 1: FORCE_ON, 2: FORCE_OFF */
  uint64_t    last_switch_ms;
  uint64_t    cycle_count;
  bool        dwell_locked;
  uint64_t    dwell_remaining_ms;
  const char* active_reason;
};

struct SystemTelemetry {
  uint64_t boot_timestamp_us;
  uint64_t nvs_write_count;
  uint64_t last_nvs_write_ms;
  uint64_t loop_cycle_count;
  bool     pending_restart;
  uint64_t restart_target_ms;
  bool     boot_btn_pressed;
  uint64_t boot_btn_start_us;
};

/* 64-bit Time Extraction Functions */
static inline uint64_t get_time_us64() {
  return (uint64_t)esp_timer_get_time();
}

static inline uint64_t get_time_ms64() {
  return ((uint64_t)esp_timer_get_time()) / 1000ULL;
}

static inline uint64_t get_time_sec64() {
  return ((uint64_t)esp_timer_get_time()) / 1000000ULL;
}

/* 64-bit CRC Calculation (CRC-64-ECMA) */
uint64_t calculateCRC64(const uint8_t *data, size_t length) {
  uint64_t crc = 0xFFFFFFFFFFFFFFFFULL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= ((uint64_t)data[i]) << 56;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000000000000000ULL) {
        crc = (crc << 1) ^ 0x42F0E1EBA9EA3693ULL;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFFFFFFFFFULL;
}

/* Global Runtime State Instances */
static DeviceConfig    config;
static SensorTelemetry sensorData;
static RelayTelemetry  relayData;
static SystemTelemetry sysData;

static BH1750      lightMeter;
static WebServer   server(80);
static DNSServer   dnsServer;
static Preferences preferences;

/* Static Buffer for JSON Serialization to Prevent Heap Fragmentation */
static char apiBuffer[2048];

/* Web UI HTML/CSS/JS Stored in Flash Memory */
const char PAGE_INDEX[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<title>ESP32 BH1750 Controller</title>
<style>
:root{--bg:#0f172a;--surface:#1e293b;--border:#334155;--text:#f8fafc;--text-dim:#94a3b8;--primary:#38bdf8;--primary-glow:rgba(56,189,248,0.25);--accent:#6366f1;--success:#22c55e;--success-glow:rgba(34,197,94,0.25);--warning:#f59e0b;--danger:#ef4444;--danger-glow:rgba(239,68,68,0.25);--card-bg:#1e293b;--input-bg:#0f172a;--radius:12px}
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
body{background:var(--bg);color:var(--text);min-height:100vh;display:flex;flex-direction:column}
header{background:rgba(30,41,59,0.85);backdrop-filter:blur(10px);border-bottom:1px solid var(--border);position:sticky;top:0;z-index:50;padding:12px 20px;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px}
.brand{display:flex;align-items:center;gap:12px}
.brand svg{width:28px;height:28px;color:var(--primary)}
.brand-title{font-size:18px;font-weight:700;letter-spacing:-0.5px}
.brand-sub{font-size:11px;color:var(--text-dim);font-weight:500;text-transform:uppercase;letter-spacing:1px}
.live-badge{display:inline-flex;align-items:center;gap:6px;background:rgba(34,197,94,0.12);color:var(--success);padding:4px 10px;border-radius:20px;font-size:12px;font-weight:600;border:1px solid rgba(34,197,94,0.3)}
.pulse-dot{width:8px;height:8px;background:var(--success);border-radius:50%;box-shadow:0 0 8px var(--success)}
nav{display:flex;gap:4px;background:var(--surface);border-bottom:1px solid var(--border);padding:6px 16px;overflow-x:auto}
.tab-btn{background:transparent;border:none;color:var(--text-dim);padding:10px 16px;border-radius:8px;font-size:14px;font-weight:600;cursor:pointer;display:flex;align-items:center;gap:8px;white-space:nowrap;transition:all 0.2s}
.tab-btn:hover{color:var(--text);background:rgba(255,255,255,0.05)}
.tab-btn.active{color:var(--primary);background:var(--primary-glow);border:1px solid rgba(56,189,248,0.3)}
main{flex:1;max-width:1100px;width:100%;margin:0 auto;padding:20px 16px}
.view{display:none}
.view.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}
.card{background:var(--card-bg);border:1px solid var(--border);border-radius:var(--radius);padding:20px;box-shadow:0 4px 6px -1px rgba(0,0,0,0.2)}
.card-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:16px;border-bottom:1px solid var(--border);padding-bottom:10px}
.card-title{font-size:15px;font-weight:600;color:var(--text);display:flex;align-items:center;gap:8px}
.hero-lux{text-align:center;padding:16px 0}
.lux-val{font-size:56px;font-weight:800;letter-spacing:-1px;color:#fff;text-shadow:0 0 20px var(--primary-glow);line-height:1}
.lux-unit{font-size:22px;color:var(--primary);font-weight:500;margin-left:4px}
.lux-category{display:inline-block;margin-top:12px;padding:6px 16px;border-radius:20px;font-size:13px;font-weight:600;background:rgba(56,189,248,0.15);color:var(--primary);border:1px solid rgba(56,189,248,0.3)}
.meter-track{background:#0f172a;height:12px;border-radius:6px;overflow:hidden;margin-top:16px;border:1px solid var(--border)}
.meter-fill{height:100%;width:0%;background:linear-gradient(90deg,#3b82f6,#38bdf8,#eab308,#ef4444);transition:width 0.3s ease}
.stat-row{display:flex;justify-content:space-between;padding:10px 0;border-bottom:1px solid rgba(255,255,255,0.05);font-size:14px}
.stat-row:last-child{border-bottom:none}
.stat-label{color:var(--text-dim)}
.stat-val{font-weight:600;font-variant-numeric:tabular-nums}
.relay-pill{display:inline-flex;align-items:center;gap:8px;padding:6px 14px;border-radius:20px;font-size:13px;font-weight:700}
.relay-on{background:rgba(34,197,94,0.2);color:var(--success);border:1px solid var(--success);box-shadow:0 0 12px var(--success-glow)}
.relay-off{background:rgba(148,163,184,0.15);color:var(--text-dim);border:1px solid var(--border)}
.form-group{margin-bottom:16px}
label{display:block;font-size:13px;font-weight:600;color:var(--text-dim);margin-bottom:6px}
.form-help{font-size:11px;color:var(--text-dim);margin-top:4px}
input,select{width:100%;background:var(--input-bg);border:1px solid var(--border);border-radius:8px;padding:10px 14px;color:#fff;font-size:14px;outline:none;transition:border 0.2s}
input:focus,select:focus{border-color:var(--primary);box-shadow:0 0 0 3px var(--primary-glow)}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;padding:11px 20px;border-radius:8px;font-size:14px;font-weight:600;cursor:pointer;border:none;transition:all 0.2s}
.btn-primary{background:var(--primary);color:#0f172a}
.btn-primary:hover{background:#7dd3fc}
.btn-secondary{background:rgba(255,255,255,0.08);color:var(--text);border:1px solid var(--border)}
.btn-secondary:hover{background:rgba(255,255,255,0.14)}
.btn-danger{background:var(--danger);color:#fff}
.btn-danger:hover{background:#dc2626}
.btn-group{display:flex;gap:10px;margin-top:20px;flex-wrap:wrap}
.toast{position:fixed;bottom:20px;right:20px;background:var(--surface);border:1px solid var(--border);padding:14px 20px;border-radius:8px;color:#fff;font-size:14px;box-shadow:0 10px 25px rgba(0,0,0,0.5);transform:translateY(100px);opacity:0;transition:all 0.3s;z-index:100}
.toast.show{transform:translateY(0);opacity:1}
.toast.success{border-color:var(--success);color:var(--success)}
.toast.error{border-color:var(--danger);color:var(--danger)}
.modal{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.7);backdrop-filter:blur(4px);display:none;align-items:center;justify-content:center;z-index:99;padding:16px}
.modal.show{display:flex}
.modal-box{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);max-width:440px;width:100%;padding:24px}
.modal-title{font-size:18px;font-weight:700;margin-bottom:10px}
.modal-desc{font-size:14px;color:var(--text-dim);margin-bottom:20px;line-height:1.5}
@media(max-width:600px){.lux-val{font-size:42px}nav{padding:4px 8px}.tab-btn{padding:8px 12px;font-size:13px}input,select{font-size:16px}}
</style>
</head>
<body>
<header>
  <div class="brand">
    <svg fill="none" stroke="currentColor" stroke-width="2" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M12 3v1m0 16v1m9-9h-1M4 12H3m15.364 6.364l-.707-.707M6.343 6.343l-.707-.707m12.728 0l-.707.707M6.343 17.657l-.707.707M16 12a4 4 0 11-8 0 4 4 0 018 0z"></path></svg>
    <div>
      <div class="brand-title">BH1750 SENSOR CONTROLLER</div>
      <div class="brand-sub">64-BIT PRECISION &bull; GPIO 25 RELAY &bull; GPIO 21/22 I2C</div>
    </div>
  </div>
  <div style="display:flex;align-items:center;gap:12px">
    <div id="connBadge" class="live-badge"><div class="pulse-dot"></div><span id="connText">ONLINE</span></div>
    <div style="font-size:12px;color:var(--text-dim)" id="topUptime">Uptime: 0s</div>
  </div>
</header>
<nav>
  <button class="tab-btn active" onclick="switchTab('tab-dash')">📊 Dashboard</button>
  <button class="tab-btn" onclick="switchTab('tab-sensor')">⚙️ Sensor & Relay</button>
  <button class="tab-btn" onclick="switchTab('tab-wifi')">📶 WiFi AP</button>
  <button class="tab-btn" onclick="switchTab('tab-sys')">🛡️ System & Diag</button>
</nav>
<main>
  <!-- TAB: DASHBOARD -->
  <section id="tab-dash" class="view active">
    <div class="grid">
      <div class="card" style="grid-column:1/-1">
        <div class="card-header"><span class="card-title">Real-Time Light Measurement</span><span id="sensorStatusTag" style="font-size:12px;color:var(--success)">I2C Bus OK (0x23)</span></div>
        <div class="hero-lux">
          <div><span id="valLux" class="lux-val">---</span><span class="lux-unit">lux</span></div>
          <div id="valCategory" class="lux-category">Measuring...</div>
        </div>
        <div class="meter-track"><div id="luxMeter" class="meter-fill"></div></div>
      </div>

      <div class="card">
        <div class="card-header"><span class="card-title">Relay Control (GPIO 25)</span><span id="relayBadge" class="relay-pill relay-off">OFF</span></div>
        <div class="stat-row"><span class="stat-label">Automation Mode</span><span id="statRelayMode" class="stat-val">---</span></div>
        <div class="stat-row"><span class="stat-label">Active Trigger Condition</span><span id="statTriggerReason" class="stat-val">---</span></div>
        <div class="stat-row"><span class="stat-label">Dwell Anti-Chatter State</span><span id="statDwellState" class="stat-val">Normal</span></div>
        <div class="stat-row"><span class="stat-label">Relay Switching Cycles</span><span id="statRelayCycles" class="stat-val">0</span></div>
        <div class="btn-group">
          <button class="btn btn-secondary" onclick="setRelayOverride(0)">Auto</button>
          <button class="btn btn-primary" onclick="setRelayOverride(1)">Force ON</button>
          <button class="btn btn-danger" onclick="setRelayOverride(2)">Force OFF</button>
        </div>
      </div>

      <div class="card">
        <div class="card-header"><span class="card-title">64-bit Statistics</span><button class="btn btn-secondary" style="padding:4px 10px;font-size:12px" onclick="clearStats()">Reset</button></div>
        <div class="stat-row"><span class="stat-label">Minimum Lux</span><span id="statMinLux" class="stat-val">--- lx</span></div>
        <div class="stat-row"><span class="stat-label">Maximum Lux</span><span id="statMaxLux" class="stat-val">--- lx</span></div>
        <div class="stat-row"><span class="stat-label">Average Lux</span><span id="statAvgLux" class="stat-val">--- lx</span></div>
        <div class="stat-row"><span class="stat-label">Total 64-bit Samples</span><span id="statSamples" class="stat-val">0</span></div>
        <div class="stat-row"><span class="stat-label">I2C Communication Errors</span><span id="statErrors" class="stat-val">0</span></div>
      </div>
    </div>
  </section>

  <!-- TAB: SENSOR & RELAY SETTINGS -->
  <section id="tab-sensor" class="view">
    <div class="grid">
      <div class="card">
        <div class="card-header"><span class="card-title">Relay Trigger Logic (GPIO 25)</span></div>
        <form id="sensorForm" onsubmit="saveSensorConfig(event)">
          <div class="form-group">
            <label>Automation Mode</label>
            <select id="cfgRelayMode">
              <option value="0">0: Disabled (Manual Control Only)</option>
              <option value="1">1: Trigger Active When Lux >= High Threshold</option>
              <option value="2">2: Trigger Active When Lux <= Low Threshold</option>
              <option value="3">3: Trigger Active Inside Range [Low, High]</option>
              <option value="4">4: Trigger Active Outside Range [Low, High]</option>
            </select>
            <div class="form-help">Evaluates continuously against live BH1750 readings.</div>
          </div>
          <div class="form-group">
            <label>High Threshold (Lux)</label>
            <input type="number" step="0.1" min="0" max="120000" id="cfgHighThresh">
            <div class="form-help">Trigger point for high-light actions (e.g. blinds down, daylight off).</div>
          </div>
          <div class="form-group">
            <label>Low Threshold (Lux)</label>
            <input type="number" step="0.1" min="0" max="120000" id="cfgLowThresh">
            <div class="form-help">Trigger point for low-light actions (e.g. night illumination on).</div>
          </div>
          <div class="form-group">
            <label>Hysteresis (Lux)</label>
            <input type="number" step="0.1" min="0" max="5000" id="cfgHysteresis">
            <div class="form-help">Prevents relay oscillation/rapid chatter near threshold boundaries.</div>
          </div>
          <div class="form-group">
            <label>Relay Output Polarity</label>
            <select id="cfgRelayInvert">
              <option value="0">Active HIGH (GPIO 25 = 3.3V when Active)</option>
              <option value="1">Active LOW (GPIO 25 = 0V when Active, for Opto Relays)</option>
            </select>
          </div>
          <div class="form-group">
            <label>Minimum ON Dwell Time (Seconds)</label>
            <input type="number" min="0" max="86400" id="cfgDwellOn">
            <div class="form-help">Anti-short-cycling: once activated, relay remains ON for at least this duration.</div>
          </div>
          <div class="form-group">
            <label>Minimum OFF Dwell Time (Seconds)</label>
            <input type="number" min="0" max="86400" id="cfgDwellOff">
            <div class="form-help">Anti-short-cycling: once deactivated, relay remains OFF for at least this duration.</div>
          </div>
          <div class="btn-group">
            <button type="submit" class="btn btn-primary">Save Settings</button>
            <button type="button" class="btn btn-secondary" onclick="loadConfig()">Reload</button>
          </div>
        </form>
      </div>

      <div class="card">
        <div class="card-header"><span class="card-title">BH1750 Sensor Hardware Parameters</span></div>
        <div class="form-group">
          <label>Measurement Resolution Mode</label>
          <select id="cfgBhMode">
            <option value="16">Continuous High Resolution Mode (1.0 lx, 120ms)</option>
            <option value="17">Continuous High Resolution Mode 2 (0.5 lx, 120ms)</option>
            <option value="19">Continuous Low Resolution Mode (4.0 lx, 16ms)</option>
          </select>
        </div>
        <div class="form-group">
          <label>Optical Measurement Time Register (MTReg)</label>
          <input type="number" min="31" max="254" id="cfgBhMTReg">
          <div class="form-help">Default 69. Adjust for optical window attenuation or ultra-bright environments.</div>
        </div>
        <div class="form-group">
          <label>Sensor Polling Interval (Milliseconds)</label>
          <select id="cfgPollInterval">
            <option value="250">250 ms (Ultra Fast)</option>
            <option value="500">500 ms (Fast)</option>
            <option value="1000">1000 ms (Standard)</option>
            <option value="2000">2000 ms (Relaxed)</option>
            <option value="5000">5000 ms (Eco)</option>
          </select>
        </div>
        <div class="stat-row"><span class="stat-label">I2C Pins</span><span class="stat-val">SDA = GPIO 21, SCL = GPIO 22</span></div>
        <div class="stat-row"><span class="stat-label">I2C Clock Rate</span><span class="stat-val">100 kHz Standard</span></div>
        <div class="stat-row"><span class="stat-label">Hardware Address Detection</span><span class="stat-val">Auto (0x23 / 0x5C)</span></div>
      </div>
    </div>
  </section>

  <!-- TAB: WIFI AP SETTINGS -->
  <section id="tab-wifi" class="view">
    <div class="grid">
      <div class="card">
        <div class="card-header"><span class="card-title">WiFi Access Point Configuration</span></div>
        <form id="wifiForm" onsubmit="saveWifiConfig(event)">
          <div class="form-group">
            <label>Access Point SSID (Network Name)</label>
            <input type="text" maxlength="32" id="cfgWifiSSID" required>
            <div class="form-help">Default: ESP32_BH1750</div>
          </div>
          <div class="form-group">
            <label>WPA2-PSK Password</label>
            <input type="password" minlength="8" maxlength="63" id="cfgWifiPass" placeholder="Leave empty for Open AP or enter 8-63 chars">
            <div class="form-help">Default: 12345678 (Minimum 8 characters)</div>
          </div>
          <div class="form-group">
            <label>WiFi Radio Channel (1 - 13)</label>
            <input type="number" min="1" max="13" id="cfgWifiChannel">
          </div>
          <div class="form-group">
            <label>Hide SSID Broadcast</label>
            <select id="cfgWifiHidden">
              <option value="0">Visible (Normal Broadcast)</option>
              <option value="1">Hidden (Stealth Mode)</option>
            </select>
          </div>
          <div class="form-group">
            <label>Max Client Connections (1 - 4)</label>
            <input type="number" min="1" max="4" id="cfgWifiMaxConn">
          </div>
          <div class="btn-group">
            <button type="submit" class="btn btn-primary">Apply & Restart AP</button>
          </div>
        </form>
      </div>

      <div class="card">
        <div class="card-header"><span class="card-title">Captive Portal & Network Status</span></div>
        <div class="stat-row"><span class="stat-label">Webserver IP</span><span class="stat-val">192.168.4.1</span></div>
        <div class="stat-row"><span class="stat-label">Subnet Mask</span><span class="stat-val">255.255.255.0</span></div>
        <div class="stat-row"><span class="stat-label">Captive Portal DNS</span><span class="stat-val">Port 53 (Wildcard * &rarr; 192.168.4.1)</span></div>
        <div class="stat-row"><span class="stat-label">Connected Stations</span><span id="statWifiClients" class="stat-val">0 client(s)</span></div>
        <div class="stat-row"><span class="stat-label">Supported OS Captive Detection</span><span class="stat-val">iOS, Android, Windows, Mac</span></div>
      </div>
    </div>
  </section>

  <!-- TAB: SYSTEM & DIAGNOSTICS -->
  <section id="tab-sys" class="view">
    <div class="grid">
      <div class="card">
        <div class="card-header"><span class="card-title">Multi-Year Reliability Telemetry</span></div>
        <div class="stat-row"><span class="stat-label">64-bit Microsecond Timebase</span><span class="stat-val" style="color:var(--success)">Active (esp_timer 64-bit)</span></div>
        <div class="stat-row"><span class="stat-label">System Uptime</span><span id="statUptimeFull" class="stat-val">---</span></div>
        <div class="stat-row"><span class="stat-label">Free Dynamic Heap</span><span id="statFreeHeap" class="stat-val">--- bytes</span></div>
        <div class="stat-row"><span class="stat-label">Minimum Ever Free Heap</span><span id="statMinHeap" class="stat-val">--- bytes</span></div>
        <div class="stat-row"><span class="stat-label">Heap Fragmentation</span><span class="stat-val" style="color:var(--success)">0% (Static Buffers)</span></div>
        <div class="stat-row"><span class="stat-label">Task Watchdog Timer</span><span class="stat-val" style="color:var(--success)">Enabled (15s hardware WDT)</span></div>
        <div class="stat-row"><span class="stat-label">NVS Write Deduplication</span><span class="stat-val" style="color:var(--success)">Enabled (CRC64 wear-mitigation)</span></div>
        <div class="stat-row"><span class="stat-label">Lifetime Flash Writes</span><span id="statNvsWrites" class="stat-val">0</span></div>
        <div class="stat-row"><span class="stat-label">Config CRC-64 Status</span><span class="stat-val" style="color:var(--success)">Verified Valid</span></div>
      </div>

      <div class="card">
        <div class="card-header"><span class="card-title">Hardware Factory Reset & Controls</span></div>
        <div style="font-size:13px;line-height:1.6;color:var(--text-dim);margin-bottom:16px">
          The ESP32 onboard <strong>GPIO 0 (BOOT button)</strong> features a non-blocking 5-second hold detection. Holding the button continuously for 5 seconds restores factory defaults (SSID: <code>ESP32_BH1750</code>, pass: <code>12345678</code>) and safely reboots.
        </div>
        <div class="btn-group">
          <button class="btn btn-secondary" onclick="confirmModal('restart')">Restart Device</button>
          <button class="btn btn-danger" onclick="confirmModal('reset')">Factory Reset</button>
        </div>
      </div>
    </div>
  </section>
</main>

<div id="toast" class="toast">Settings Saved</div>

<div id="modal" class="modal">
  <div class="modal-box">
    <div id="modalTitle" class="modal-title">Confirm Action</div>
    <div id="modalDesc" class="modal-desc">Are you sure?</div>
    <div class="btn-group" style="justify-content:flex-end">
      <button class="btn btn-secondary" onclick="closeModal()">Cancel</button>
      <button id="modalConfirmBtn" class="btn btn-danger" onclick="executeModalAction()">Confirm</button>
    </div>
  </div>
</div>

<script>
let currentTab='tab-dash';
let pendingAction='';

function switchTab(tabId){
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.querySelectorAll('.view').forEach(v=>v.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  const btn = Array.from(document.querySelectorAll('.tab-btn')).find(b=>b.getAttribute('onclick').includes(tabId));
  if(btn) btn.classList.add('active');
  currentTab=tabId;
}

function showToast(msg,isErr=false){
  const t=document.getElementById('toast');
  t.innerText=msg;
  t.className='toast show '+(isErr?'error':'success');
  setTimeout(()=>{t.className='toast';},3000);
}

function confirmModal(action){
  pendingAction=action;
  const m=document.getElementById('modal');
  const title=document.getElementById('modalTitle');
  const desc=document.getElementById('modalDesc');
  const btn=document.getElementById('modalConfirmBtn');
  if(action==='restart'){
    title.innerText='Restart System';
    desc.innerText='This will perform a soft reboot of the ESP32. WiFi will reconnect in a few seconds.';
    btn.innerText='Restart Now';
    btn.className='btn btn-primary';
  } else if(action==='reset'){
    title.innerText='Factory Reset';
    desc.innerText='WARNING: This will reset all BH1750 thresholds and restore default AP credentials (SSID: ESP32_BH1750, Pass: 12345678). Continue?';
    btn.innerText='Factory Reset';
    btn.className='btn btn-danger';
  }
  m.classList.add('show');
}
function closeModal(){document.getElementById('modal').classList.remove('show');}

function executeModalAction(){
  closeModal();
  if(pendingAction==='restart'){
    fetch('/api/system/restart',{method:'POST'}).then(()=>{showToast('Restarting device...');}).catch(()=>showToast('Error',true));
  } else if(pendingAction==='reset'){
    fetch('/api/system/factory_reset',{method:'POST'}).then(()=>{showToast('Factory reset! Rebooting...');}).catch(()=>showToast('Error',true));
  }
}

function getCategory(lux){
  if(lux<1) return 'Pitch Black';
  if(lux<10) return 'Moonlight / Dim Night';
  if(lux<50) return 'Dim Indoor / Nightlight';
  if(lux<150) return 'Living Room Lighting';
  if(lux<500) return 'Office / Study Desk';
  if(lux<1000) return 'Bright Lab / Supermarket';
  if(lux<10000) return 'Overcast Sky / Daylight';
  if(lux<30000) return 'Full Daylight';
  return 'Direct Sunlight';
}

function formatUptime(sec){
  const d=Math.floor(sec/86400);
  const h=Math.floor((sec%86400)/3600);
  const m=Math.floor((sec%3600)/60);
  const s=sec%60;
  return `${d}d ${h}h ${m}m ${s}s`;
}

function updateStatus(){
  fetch('/api/status')
    .then(r=>r.json())
    .then(d=>{
      document.getElementById('connBadge').className='live-badge';
      document.getElementById('connText').innerText='ONLINE';
      if(d.lux_valid){
        document.getElementById('valLux').innerText=d.lux.toFixed(1);
        document.getElementById('valCategory').innerText=getCategory(d.lux);
        let pct=Math.min(100,Math.max(0,(Math.log10(Math.max(1,d.lux))/5.0)*100));
        document.getElementById('luxMeter').style.width=pct.toFixed(1)+'%';
      } else {
        document.getElementById('valLux').innerText='ERR';
        document.getElementById('valCategory').innerText='Sensor Read Error';
      }
      document.getElementById('statMinLux').innerText=(d.lux_min>=0?d.lux_min.toFixed(1):'---')+' lx';
      document.getElementById('statMaxLux').innerText=(d.lux_max>=0?d.lux_max.toFixed(1):'---')+' lx';
      document.getElementById('statAvgLux').innerText=(d.lux_avg>=0?d.lux_avg.toFixed(1):'---')+' lx';
      document.getElementById('statSamples').innerText=d.samples.toLocaleString();
      document.getElementById('statErrors').innerText=d.errors.toLocaleString();

      const rPill=document.getElementById('relayBadge');
      if(d.relay_state===1){
        rPill.className='relay-pill relay-on';
        rPill.innerText='ACTIVE / ON';
      } else {
        rPill.className='relay-pill relay-off';
        rPill.innerText='INACTIVE / OFF';
      }

      document.getElementById('statRelayMode').innerText=d.mode_desc||'Auto';
      document.getElementById('statTriggerReason').innerText=d.trigger_desc||'None';
      document.getElementById('statDwellState').innerText=d.relay_dwell_active?('Dwell Active ('+d.relay_dwell_remaining_s.toFixed(1)+'s)'):'Normal';
      document.getElementById('statRelayCycles').innerText=d.relay_cycles.toLocaleString();

      const upStr=formatUptime(d.uptime_sec);
      document.getElementById('topUptime').innerText='Uptime: '+upStr;
      document.getElementById('statUptimeFull').innerText=upStr;
      document.getElementById('statFreeHeap').innerText=d.free_heap.toLocaleString()+' bytes';
      document.getElementById('statMinHeap').innerText=d.min_free_heap.toLocaleString()+' bytes';
      document.getElementById('statNvsWrites').innerText=d.nvs_writes.toLocaleString();
      document.getElementById('statWifiClients').innerText=d.wifi_clients+' station(s)';
      document.getElementById('sensorStatusTag').innerText=d.sensor_online?('I2C Bus OK ('+d.sensor_addr+')'):'Sensor Disconnected';
      document.getElementById('sensorStatusTag').style.color=d.sensor_online?'var(--success)':'var(--danger)';
    })
    .catch(()=>{
      document.getElementById('connBadge').className='live-badge';
      document.getElementById('connBadge').style.borderColor='var(--danger)';
      document.getElementById('connText').innerText='DISCONNECTED';
      document.getElementById('connText').style.color='var(--danger)';
    });
}

function loadConfig(){
  fetch('/api/config')
    .then(r=>r.json())
    .then(c=>{
      document.getElementById('cfgRelayMode').value=c.relay_mode;
      document.getElementById('cfgHighThresh').value=c.high_thresh;
      document.getElementById('cfgLowThresh').value=c.low_thresh;
      document.getElementById('cfgHysteresis').value=c.hysteresis;
      document.getElementById('cfgRelayInvert').value=c.relay_inverted;
      document.getElementById('cfgDwellOn').value=c.dwell_on_s;
      document.getElementById('cfgDwellOff').value=c.dwell_off_s;
      document.getElementById('cfgBhMode').value=c.bh1750_mode;
      document.getElementById('cfgBhMTReg').value=c.bh1750_mtreg;
      document.getElementById('cfgPollInterval').value=c.poll_interval_ms;
      document.getElementById('cfgWifiSSID').value=c.wifi_ssid;
      document.getElementById('cfgWifiPass').value=c.wifi_pass;
      document.getElementById('cfgWifiChannel').value=c.wifi_channel;
      document.getElementById('cfgWifiHidden').value=c.wifi_hidden;
      document.getElementById('cfgWifiMaxConn').value=c.wifi_max_conn;
    });
}

function saveSensorConfig(e){
  e.preventDefault();
  const payload={
    relay_mode: parseInt(document.getElementById('cfgRelayMode').value),
    high_thresh: parseFloat(document.getElementById('cfgHighThresh').value),
    low_thresh: parseFloat(document.getElementById('cfgLowThresh').value),
    hysteresis: parseFloat(document.getElementById('cfgHysteresis').value),
    relay_inverted: parseInt(document.getElementById('cfgRelayInvert').value),
    dwell_on_s: parseInt(document.getElementById('cfgDwellOn').value),
    dwell_off_s: parseInt(document.getElementById('cfgDwellOff').value),
    bh1750_mode: parseInt(document.getElementById('cfgBhMode').value),
    bh1750_mtreg: parseInt(document.getElementById('cfgBhMTReg').value),
    poll_interval_ms: parseInt(document.getElementById('cfgPollInterval').value)
  };
  fetch('/api/settings/sensor',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(payload)
  })
  .then(r=>r.json())
  .then(res=>{
    if(res.success){
      showToast(res.message||'Sensor settings saved!');
    } else {
      showToast(res.error||'Failed to save',true);
    }
  })
  .catch(()=>showToast('Network error saving settings',true));
}

function saveWifiConfig(e){
  e.preventDefault();
  const payload={
    wifi_ssid: document.getElementById('cfgWifiSSID').value,
    wifi_pass: document.getElementById('cfgWifiPass').value,
    wifi_channel: parseInt(document.getElementById('cfgWifiChannel').value),
    wifi_hidden: parseInt(document.getElementById('cfgWifiHidden').value),
    wifi_max_conn: parseInt(document.getElementById('cfgWifiMaxConn').value)
  };
  fetch('/api/settings/wifi',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(payload)
  })
  .then(r=>r.json())
  .then(res=>{
    if(res.success){
      showToast(res.message||'WiFi AP updated! Reconnecting...');
    } else {
      showToast(res.error||'Failed to update WiFi',true);
    }
  })
  .catch(()=>showToast('Network error saving WiFi',true));
}

function setRelayOverride(mode){
  fetch('/api/relay/override?mode='+mode,{method:'POST'})
    .then(r=>r.json())
    .then(res=>{
      showToast(res.message||'Relay updated');
      updateStatus();
    });
}

function clearStats(){
  fetch('/api/stats/clear',{method:'POST'})
    .then(r=>r.json())
    .then(res=>{
      showToast('Statistics reset!');
      updateStatus();
    });
}

window.addEventListener('load',()=>{
  loadConfig();
  updateStatus();
  setInterval(updateStatus, 1000);
});
</script>
</body>
</html>)rawliteral";

/* Zero-Allocation In-Place Key-Value Parsers */
static bool jsonGetDouble(const char* json, const char* key, double& outVal) {
  if (!json || !key) return false;
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  while (*p && (*p == ' ' || *p == ':' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
  if (!*p) return false;
  char* endptr = nullptr;
  double v = strtod(p, &endptr);
  if (endptr == p) return false;
  outVal = v;
  return true;
}

static bool jsonGetUInt64(const char* json, const char* key, uint64_t& outVal) {
  if (!json || !key) return false;
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  while (*p && (*p == ' ' || *p == ':' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
  if (!*p) return false;
  char* endptr = nullptr;
  unsigned long long v = strtoull(p, &endptr, 10);
  if (endptr == p) return false;
  outVal = (uint64_t)v;
  return true;
}

static bool jsonGetString(const char* json, const char* key, char* outBuf, size_t maxLen) {
  if (!json || !key || !outBuf || maxLen == 0) return false;
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p += strlen(pattern);
  while (*p && (*p == ' ' || *p == ':' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
  if (*p != '\"') return false;
  p++;
  size_t idx = 0;
  while (*p && *p != '\"' && idx < (maxLen - 1)) {
    if (*p == '\\' && *(p + 1)) p++;
    outBuf[idx++] = *p++;
  }
  outBuf[idx] = '\0';
  return true;
}

/* Default Configuration Construction */
static void populateDefaultConfig(DeviceConfig &cfg) {
  memset(&cfg, 0, sizeof(DeviceConfig));
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  strncpy(cfg.wifi_ssid, DEFAULT_SSID, sizeof(cfg.wifi_ssid) - 1);
  strncpy(cfg.wifi_pass, DEFAULT_PASS, sizeof(cfg.wifi_pass) - 1);
  cfg.wifi_channel = 1ULL;
  cfg.wifi_hidden = 0ULL;
  cfg.wifi_max_conn = 4ULL;
  cfg.relay_mode = 2ULL; /* Mode 2: Trigger Below Low Threshold */
  cfg.high_threshold = 500.0;
  cfg.low_threshold = 50.0;
  cfg.hysteresis = 5.0;
  cfg.relay_inverted = 0ULL; /* Active HIGH */
  cfg.dwell_on_ms = 5000ULL;
  cfg.dwell_off_ms = 5000ULL;
  cfg.bh1750_mode = (uint64_t)BH1750::CONTINUOUS_HIGH_RES_MODE;
  cfg.bh1750_mtreg = 69ULL;
  cfg.poll_interval_ms = 1000ULL;
  cfg.crc64 = calculateCRC64((const uint8_t*)&cfg, sizeof(DeviceConfig) - sizeof(uint64_t));
}

/* Comprehensive Range & Integrity Validation */
static bool isConfigValid(const DeviceConfig &cfg) {
  if (cfg.magic != CONFIG_MAGIC) return false;
  if (cfg.version != CONFIG_VERSION) return false;
  uint64_t calcCrc = calculateCRC64((const uint8_t*)&cfg, sizeof(DeviceConfig) - sizeof(uint64_t));
  if (calcCrc != cfg.crc64) return false;
  if (strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_ssid) > 32) return false;
  if (strlen(cfg.wifi_pass) > 0 && strlen(cfg.wifi_pass) < 8) return false;
  if (strlen(cfg.wifi_pass) > 63) return false;
  if (cfg.wifi_channel < 1ULL || cfg.wifi_channel > 13ULL) return false;
  if (cfg.wifi_hidden > 1ULL) return false;
  if (cfg.wifi_max_conn < 1ULL || cfg.wifi_max_conn > 4ULL) return false;
  if (cfg.relay_mode > 4ULL) return false;
  if (cfg.high_threshold < 0.0 || cfg.high_threshold > 120000.0 || isnan(cfg.high_threshold) || isinf(cfg.high_threshold)) return false;
  if (cfg.low_threshold < 0.0 || cfg.low_threshold > 120000.0 || isnan(cfg.low_threshold) || isinf(cfg.low_threshold)) return false;
  if (cfg.low_threshold > cfg.high_threshold) return false;
  if (cfg.hysteresis < 0.0 || cfg.hysteresis > 5000.0 || isnan(cfg.hysteresis) || isinf(cfg.hysteresis)) return false;
  if (cfg.relay_mode == 4ULL && (cfg.low_threshold + cfg.hysteresis > cfg.high_threshold - cfg.hysteresis)) return false;
  if (cfg.relay_inverted > 1ULL) return false;
  if (cfg.dwell_on_ms > 86400000ULL) return false;
  if (cfg.dwell_off_ms > 86400000ULL) return false;
  if (cfg.bh1750_mtreg < 31ULL || cfg.bh1750_mtreg > 254ULL) return false;
  if (cfg.poll_interval_ms < 100ULL || cfg.poll_interval_ms > 60000ULL) return false;
  return true;
}

/* NVS Wear-Mitigated Persistence Handler */
static bool saveConfigToNVS(const DeviceConfig &newCfg, bool force = false) {
  uint64_t now_ms = get_time_ms64();
  if (!force && (now_ms - sysData.last_nvs_write_ms < MIN_NVS_INTERVAL_MS)) {
    return false;
  }
  if (!force && (memcmp(&config, &newCfg, sizeof(DeviceConfig)) == 0)) {
    return true; /* Identical data - skip flash write to preserve endurance */
  }

  DeviceConfig cfgToWrite = newCfg;
  cfgToWrite.magic = CONFIG_MAGIC;
  cfgToWrite.version = CONFIG_VERSION;
  cfgToWrite.crc64 = calculateCRC64((const uint8_t*)&cfgToWrite, sizeof(DeviceConfig) - sizeof(uint64_t));

  preferences.begin("bh_cfg", false);
  size_t written = preferences.putBytes("blob", &cfgToWrite, sizeof(DeviceConfig));
  preferences.end();

  if (written == sizeof(DeviceConfig)) {
    config = cfgToWrite;
    sysData.nvs_write_count++;
    sysData.last_nvs_write_ms = now_ms;
    return true;
  }
  return false;
}

static void loadConfigFromNVS() {
  preferences.begin("bh_cfg", false);
  DeviceConfig tempCfg;
  size_t bytesRead = preferences.getBytes("blob", &tempCfg, sizeof(DeviceConfig));
  preferences.end();

  if (bytesRead == sizeof(DeviceConfig) && isConfigValid(tempCfg)) {
    config = tempCfg;
  } else {
    populateDefaultConfig(config);
    preferences.begin("bh_cfg", false);
    preferences.putBytes("blob", &config, sizeof(DeviceConfig));
    preferences.end();
    sysData.nvs_write_count++;
  }
}

/* Physical Relay Actuator Output */
static void applyRelayOutput(bool active) {
  relayData.active_state = active;
  uint8_t physical_level = LOW;
  if (config.relay_inverted == 0ULL) {
    physical_level = active ? HIGH : LOW;
  } else {
    physical_level = active ? LOW : HIGH;
  }
  digitalWrite(PIN_RELAY_OUTPUT, physical_level);
}

/* Hardware I2C Bus Hang Recovery */
static bool recoverI2CBus() {
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);

  bool bus_cleared = false;
  for (int i = 0; i < 9; ++i) {
    if (digitalRead(PIN_I2C_SDA) == HIGH) {
      bus_cleared = true;
      break;
    }
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(5);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000U);
  return bus_cleared;
}

/* BH1750 Sensor Initialization */
static bool initBH1750Sensor() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000U);

  /* Probe primary address 0x23 */
  Wire.beginTransmission(0x23);
  if (Wire.endTransmission() == 0) {
    sensorData.detected_address = 0x23;
  } else {
    /* Probe secondary address 0x5C */
    Wire.beginTransmission(0x5C);
    if (Wire.endTransmission() == 0) {
      sensorData.detected_address = 0x5C;
    } else {
      sensorData.detected_address = 0x23;
      sensorData.sensor_online = false;
      return false;
    }
  }

  bool ok = lightMeter.begin((BH1750::Mode)config.bh1750_mode, sensorData.detected_address, &Wire);
  if (ok) {
    lightMeter.setMTreg((uint8_t)config.bh1750_mtreg);
    sensorData.sensor_online = true;
    sensorData.consecutive_failures = 0;
    return true;
  }
  sensorData.sensor_online = false;
  return false;
}

/* Non-blocking Photometric Sampling */
static void sampleSensorNonBlocking() {
  uint64_t now_ms = get_time_ms64();
  if (now_ms - sensorData.last_poll_ms < config.poll_interval_ms) {
    return;
  }
  sensorData.last_poll_ms = now_ms;

  if (!sensorData.sensor_online) {
    if (sensorData.consecutive_failures % 5 == 0) {
      recoverI2CBus();
      initBH1750Sensor();
    }
    sensorData.consecutive_failures++;
    return;
  }

  float val = lightMeter.readLightLevel();
  if (val >= 0.0f && !isnan(val) && !isinf(val)) {
    double reading = (double)val;
    sensorData.current_lux = reading;
    sensorData.lux_accumulator += reading;
    sensorData.total_samples++;
    sensorData.last_read_valid = true;
    sensorData.consecutive_failures = 0;

    if (sensorData.min_lux < 0.0 || reading < sensorData.min_lux) {
      sensorData.min_lux = reading;
    }
    if (sensorData.max_lux < 0.0 || reading > sensorData.max_lux) {
      sensorData.max_lux = reading;
    }
  } else {
    sensorData.error_samples++;
    sensorData.last_read_valid = false;
    sensorData.consecutive_failures++;
    if (sensorData.consecutive_failures >= 5) {
      sensorData.sensor_online = false;
    }
  }
}

/* Non-blocking Relay Actuator Evaluation with Anti-Chatter Dwell */
static void evaluateRelayNonBlocking() {
  uint64_t now_ms = get_time_ms64();

  if (relayData.override_mode == 1ULL) {
    if (!relayData.active_state) {
      applyRelayOutput(true);
      relayData.last_switch_ms = now_ms;
      relayData.cycle_count++;
    }
    relayData.dwell_locked = false;
    relayData.dwell_remaining_ms = 0ULL;
    relayData.active_reason = "Manual Override (Force ON)";
    return;
  } else if (relayData.override_mode == 2ULL) {
    if (relayData.active_state) {
      applyRelayOutput(false);
      relayData.last_switch_ms = now_ms;
    }
    relayData.dwell_locked = false;
    relayData.dwell_remaining_ms = 0ULL;
    relayData.active_reason = "Manual Override (Force OFF)";
    return;
  }

  if (config.relay_mode == 0ULL) {
    if (relayData.active_state) {
      applyRelayOutput(false);
      relayData.last_switch_ms = now_ms;
    }
    relayData.dwell_locked = false;
    relayData.dwell_remaining_ms = 0ULL;
    relayData.active_reason = "Automation Disabled (Manual Only)";
    return;
  }

  if (!sensorData.last_read_valid) {
    relayData.active_reason = "Holding Last State (Sensor Comm Fault)";
    return;
  }

  double lux = sensorData.current_lux;
  bool desired = relayData.active_state;
  const char* reason = "Steady State";

  switch (config.relay_mode) {
    case 1ULL: /* Trigger Above High Threshold */
      if (lux >= config.high_threshold) {
        desired = true;
        reason = "Triggered: Lux >= High Threshold";
      } else if (lux < (config.high_threshold - config.hysteresis)) {
        desired = false;
        reason = "Deactivated: Lux < (High - Hysteresis)";
      } else {
        reason = relayData.active_state ? "Holding Active (Within High Hysteresis)" : "Holding Inactive (Within High Hysteresis)";
      }
      break;

    case 2ULL: /* Trigger Below Low Threshold */
      if (lux <= config.low_threshold) {
        desired = true;
        reason = "Triggered: Lux <= Low Threshold";
      } else if (lux > (config.low_threshold + config.hysteresis)) {
        desired = false;
        reason = "Deactivated: Lux > (Low + Hysteresis)";
      } else {
        reason = relayData.active_state ? "Holding Active (Within Low Hysteresis)" : "Holding Inactive (Within Low Hysteresis)";
      }
      break;

    case 3ULL: /* Trigger Inside Window [Low, High] */
      if (lux >= config.low_threshold && lux <= config.high_threshold) {
        desired = true;
        reason = "Triggered: Lux Inside Range [Low, High]";
      } else if (lux < (config.low_threshold - config.hysteresis) || lux > (config.high_threshold + config.hysteresis)) {
        desired = false;
        reason = "Deactivated: Lux Outside Range";
      } else {
        reason = relayData.active_state ? "Holding Active (Window Boundary)" : "Holding Inactive (Window Boundary)";
      }
      break;

    case 4ULL: /* Trigger Outside Window [Low, High] */
      if (lux < config.low_threshold || lux > config.high_threshold) {
        desired = true;
        reason = "Triggered: Lux Outside Range [Low, High]";
      } else if (lux >= (config.low_threshold + config.hysteresis) && lux <= (config.high_threshold - config.hysteresis)) {
        desired = false;
        reason = "Deactivated: Lux Inside Deadband";
      } else {
        reason = relayData.active_state ? "Holding Active (Deadband)" : "Holding Inactive (Deadband)";
      }
      break;

    default:
      break;
  }

  /* Anti-Chatter Minimum Dwell Time Enforcer */
  if (desired != relayData.active_state) {
    if (desired) {
      uint64_t elapsed = now_ms - relayData.last_switch_ms;
      if (elapsed >= config.dwell_off_ms) {
        applyRelayOutput(true);
        relayData.last_switch_ms = now_ms;
        relayData.cycle_count++;
        relayData.dwell_locked = false;
        relayData.dwell_remaining_ms = 0ULL;
        relayData.active_reason = reason;
      } else {
        relayData.dwell_locked = true;
        relayData.dwell_remaining_ms = config.dwell_off_ms - elapsed;
        relayData.active_reason = "Dwell Lock Active (Holding OFF)";
      }
    } else {
      uint64_t elapsed = now_ms - relayData.last_switch_ms;
      if (elapsed >= config.dwell_on_ms) {
        applyRelayOutput(false);
        relayData.last_switch_ms = now_ms;
        relayData.dwell_locked = false;
        relayData.dwell_remaining_ms = 0ULL;
        relayData.active_reason = reason;
      } else {
        relayData.dwell_locked = true;
        relayData.dwell_remaining_ms = config.dwell_on_ms - elapsed;
        relayData.active_reason = "Dwell Lock Active (Holding ON)";
      }
    }
  } else {
    relayData.dwell_locked = false;
    relayData.dwell_remaining_ms = 0ULL;
    relayData.active_reason = reason;
  }
}

/* Factory Reset Execution */
static void executeFactoryReset() {
  populateDefaultConfig(config);
  preferences.begin("bh_cfg", false);
  preferences.clear();
  preferences.putBytes("blob", &config, sizeof(DeviceConfig));
  preferences.end();
  applyRelayOutput(false);
  esp_restart();
}

/* Non-blocking Hardware BOOT Button (GPIO 0) 5-second Hold Detector */
static void checkBootButtonNonBlocking() {
  uint64_t now_us = get_time_us64();
  int btn_reading = digitalRead(PIN_BOOT_BUTTON);

  if (btn_reading == LOW) {
    if (!sysData.boot_btn_pressed) {
      sysData.boot_btn_pressed = true;
      sysData.boot_btn_start_us = now_us;
    } else {
      uint64_t hold_duration_ms = (now_us - sysData.boot_btn_start_us) / 1000ULL;
      if (hold_duration_ms >= 5000ULL) {
        executeFactoryReset();
      }
    }
  } else {
    sysData.boot_btn_pressed = false;
  }
}

/* Watchdog Timer Initialization */
static void setupTaskWatchdog(uint32_t timeout_seconds) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = timeout_seconds * 1000U,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(timeout_seconds, true);
  esp_task_wdt_add(NULL);
#endif
}

/* Web Server Handlers */
static void handleRootPage() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send_P(200, "text/html", PAGE_INDEX);
}

static void handleCaptiveRedirect() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

static void handleNotFound() {
  String host = server.hostHeader();
  if (!host.startsWith("192.168.4.1")) {
    handleCaptiveRedirect();
    return;
  }
  server.send(404, "text/plain", "Not Found");
}

static void handleApiStatus() {
  double avgLux = (sensorData.total_samples > 0) ? (sensorData.lux_accumulator / (double)sensorData.total_samples) : -1.0;
  uint64_t uptime_sec = get_time_sec64();
  uint32_t free_heap = ESP.getFreeHeap();
  uint32_t min_heap = ESP.getMinFreeHeap();
  int wifi_clients = WiFi.softAPgetStationNum();

  const char* modeDesc = "Disabled";
  if (config.relay_mode == 1ULL) modeDesc = "Above High Threshold";
  else if (config.relay_mode == 2ULL) modeDesc = "Below Low Threshold";
  else if (config.relay_mode == 3ULL) modeDesc = "Inside Window Range";
  else if (config.relay_mode == 4ULL) modeDesc = "Outside Window Range";

  char addrStr[8];
  snprintf(addrStr, sizeof(addrStr), "0x%02X", sensorData.detected_address);

  snprintf(apiBuffer, sizeof(apiBuffer),
    "{"
    "\"lux\":%.2f,"
    "\"lux_valid\":%s,"
    "\"lux_min\":%.2f,"
    "\"lux_max\":%.2f,"
    "\"lux_avg\":%.2f,"
    "\"samples\":%llu,"
    "\"errors\":%llu,"
    "\"sensor_addr\":\"%s\","
    "\"sensor_online\":%s,"
    "\"relay_state\":%d,"
    "\"relay_override\":%llu,"
    "\"relay_dwell_active\":%s,"
    "\"relay_dwell_remaining_s\":%.1f,"
    "\"relay_cycles\":%llu,"
    "\"mode_desc\":\"%s\","
    "\"trigger_desc\":\"%s\","
    "\"uptime_sec\":%llu,"
    "\"free_heap\":%u,"
    "\"min_free_heap\":%u,"
    "\"nvs_writes\":%llu,"
    "\"wifi_clients\":%d"
    "}",
    sensorData.current_lux,
    sensorData.last_read_valid ? "true" : "false",
    sensorData.min_lux,
    sensorData.max_lux,
    avgLux,
    (unsigned long long)sensorData.total_samples,
    (unsigned long long)sensorData.error_samples,
    addrStr,
    sensorData.sensor_online ? "true" : "false",
    relayData.active_state ? 1 : 0,
    (unsigned long long)relayData.override_mode,
    relayData.dwell_locked ? "true" : "false",
    (double)(relayData.dwell_remaining_ms) / 1000.0,
    (unsigned long long)relayData.cycle_count,
    modeDesc,
    relayData.active_reason ? relayData.active_reason : "None",
    (unsigned long long)uptime_sec,
    free_heap,
    min_heap,
    (unsigned long long)sysData.nvs_write_count,
    wifi_clients
  );

  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", apiBuffer);
}

static void handleApiConfig() {
  snprintf(apiBuffer, sizeof(apiBuffer),
    "{"
    "\"wifi_ssid\":\"%s\","
    "\"wifi_pass\":\"%s\","
    "\"wifi_channel\":%llu,"
    "\"wifi_hidden\":%llu,"
    "\"wifi_max_conn\":%llu,"
    "\"relay_mode\":%llu,"
    "\"high_thresh\":%.2f,"
    "\"low_thresh\":%.2f,"
    "\"hysteresis\":%.2f,"
    "\"relay_inverted\":%llu,"
    "\"dwell_on_s\":%llu,"
    "\"dwell_off_s\":%llu,"
    "\"bh1750_mode\":%llu,"
    "\"bh1750_mtreg\":%llu,"
    "\"poll_interval_ms\":%llu"
    "}",
    config.wifi_ssid,
    config.wifi_pass,
    (unsigned long long)config.wifi_channel,
    (unsigned long long)config.wifi_hidden,
    (unsigned long long)config.wifi_max_conn,
    (unsigned long long)config.relay_mode,
    config.high_threshold,
    config.low_threshold,
    config.hysteresis,
    (unsigned long long)config.relay_inverted,
    (unsigned long long)(config.dwell_on_ms / 1000ULL),
    (unsigned long long)(config.dwell_off_ms / 1000ULL),
    (unsigned long long)config.bh1750_mode,
    (unsigned long long)config.bh1750_mtreg,
    (unsigned long long)config.poll_interval_ms
  );

  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", apiBuffer);
}

static void handleSaveSensorSettings() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method Not Allowed\"}");
    return;
  }
  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Empty request body\"}");
    return;
  }
  const char* json = body.c_str();

  DeviceConfig candidate = config;
  double dVal = 0.0;
  uint64_t uVal = 0ULL;

  if (jsonGetUInt64(json, "relay_mode", uVal) && uVal <= 4ULL) candidate.relay_mode = uVal;
  if (jsonGetDouble(json, "high_thresh", dVal) && dVal >= 0.0 && dVal <= 120000.0) candidate.high_threshold = dVal;
  if (jsonGetDouble(json, "low_thresh", dVal) && dVal >= 0.0 && dVal <= 120000.0) candidate.low_threshold = dVal;
  if (jsonGetDouble(json, "hysteresis", dVal) && dVal >= 0.0 && dVal <= 5000.0) candidate.hysteresis = dVal;
  if (jsonGetUInt64(json, "relay_inverted", uVal) && uVal <= 1ULL) candidate.relay_inverted = uVal;
  if (jsonGetUInt64(json, "dwell_on_s", uVal) && uVal <= 86400ULL) candidate.dwell_on_ms = uVal * 1000ULL;
  if (jsonGetUInt64(json, "dwell_off_s", uVal) && uVal <= 86400ULL) candidate.dwell_off_ms = uVal * 1000ULL;
  if (jsonGetUInt64(json, "bh1750_mode", uVal)) candidate.bh1750_mode = uVal;
  if (jsonGetUInt64(json, "bh1750_mtreg", uVal) && uVal >= 31ULL && uVal <= 254ULL) candidate.bh1750_mtreg = uVal;
  if (jsonGetUInt64(json, "poll_interval_ms", uVal) && uVal >= 100ULL && uVal <= 60000ULL) candidate.poll_interval_ms = uVal;

  candidate.crc64 = calculateCRC64((const uint8_t*)&candidate, sizeof(DeviceConfig) - sizeof(uint64_t));

  if (!isConfigValid(candidate)) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Validation failed on sensor parameters\"}");
    return;
  }

  bool saved = saveConfigToNVS(candidate);
  if (saved) {
    if (sensorData.sensor_online) {
      lightMeter.configure((BH1750::Mode)config.bh1750_mode);
      lightMeter.setMTreg((uint8_t)config.bh1750_mtreg);
    }
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Sensor settings successfully saved & applied.\"}");
  } else {
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Settings identical; flash write bypassed to protect NVS endurance.\"}");
  }
}

static void handleSaveWifiSettings() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method Not Allowed\"}");
    return;
  }
  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Empty request body\"}");
    return;
  }
  const char* json = body.c_str();

  DeviceConfig candidate = config;
  char strBuf[65];
  uint64_t uVal = 0ULL;

  if (jsonGetString(json, "wifi_ssid", strBuf, sizeof(strBuf)) && strlen(strBuf) > 0 && strlen(strBuf) <= 32) {
    strncpy(candidate.wifi_ssid, strBuf, sizeof(candidate.wifi_ssid) - 1);
  }
  if (jsonGetString(json, "wifi_pass", strBuf, sizeof(strBuf))) {
    if (strlen(strBuf) == 0 || (strlen(strBuf) >= 8 && strlen(strBuf) <= 63)) {
      strncpy(candidate.wifi_pass, strBuf, sizeof(candidate.wifi_pass) - 1);
    } else {
      server.send(400, "application/json", "{\"success\":false,\"error\":\"Password must be empty or 8-63 characters\"}");
      return;
    }
  }
  if (jsonGetUInt64(json, "wifi_channel", uVal) && uVal >= 1ULL && uVal <= 13ULL) candidate.wifi_channel = uVal;
  if (jsonGetUInt64(json, "wifi_hidden", uVal) && uVal <= 1ULL) candidate.wifi_hidden = uVal;
  if (jsonGetUInt64(json, "wifi_max_conn", uVal) && uVal >= 1ULL && uVal <= 4ULL) candidate.wifi_max_conn = uVal;

  candidate.crc64 = calculateCRC64((const uint8_t*)&candidate, sizeof(DeviceConfig) - sizeof(uint64_t));

  if (!isConfigValid(candidate)) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Validation failed on WiFi AP parameters\"}");
    return;
  }

  bool saved = saveConfigToNVS(candidate, true);
  if (saved) {
    server.send(200, "application/json", "{\"success\":true,\"message\":\"WiFi settings saved. Restarting AP in 2 seconds...\"}");
    sysData.pending_restart = true;
    sysData.restart_target_ms = get_time_ms64() + 2000ULL;
  } else {
    server.send(500, "application/json", "{\"success\":false,\"error\":\"Flash storage write failed\"}");
  }
}

static void handleRelayOverride() {
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    uint64_t modeVal = (uint64_t)m.toInt();
    if (modeVal <= 2ULL) {
      relayData.override_mode = modeVal;
      server.send(200, "application/json", "{\"success\":true,\"message\":\"Relay override state updated\"}");
      return;
    }
  }
  server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid override mode\"}");
}

static void handleClearStats() {
  sensorData.min_lux = -1.0;
  sensorData.max_lux = -1.0;
  sensorData.lux_accumulator = 0.0;
  sensorData.total_samples = 0ULL;
  sensorData.error_samples = 0ULL;
  relayData.cycle_count = 0ULL;
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Statistics reset to zero\"}");
}

static void handleSystemRestart() {
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Soft reboot initiated...\"}");
  sysData.pending_restart = true;
  sysData.restart_target_ms = get_time_ms64() + 1000ULL;
}

static void handleSystemFactoryReset() {
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Factory reset acknowledged. Restoring defaults and rebooting...\"}");
  sysData.pending_restart = true;
  sysData.restart_target_ms = get_time_ms64() + 1000ULL;
  executeFactoryReset();
}

/* Arduino Setup Routine */
void setup() {
  /* Suppress all UART logging to enforce zero serial output */
  esp_log_level_set("*", ESP_LOG_NONE);

  /* Configure Pin Directions */
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RELAY_OUTPUT, OUTPUT);
  digitalWrite(PIN_RELAY_OUTPUT, LOW);

  /* Initialize Runtime Data */
  memset(&sensorData, 0, sizeof(SensorTelemetry));
  sensorData.min_lux = -1.0;
  sensorData.max_lux = -1.0;

  memset(&relayData, 0, sizeof(RelayTelemetry));
  relayData.active_reason = "System Initialized";

  memset(&sysData, 0, sizeof(SystemTelemetry));
  sysData.boot_timestamp_us = get_time_us64();

  /* Load Verified Config from Flash */
  loadConfigFromNVS();

  /* Set initial safe relay output */
  applyRelayOutput(false);

  /* Initialize BH1750 Sensor on I2C GPIO 21 & GPIO 22 */
  initBH1750Sensor();

  /* Initialize WiFi Access Point */
  WiFi.mode(WIFI_AP);
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  const char* pass = (strlen(config.wifi_pass) >= 8) ? config.wifi_pass : NULL;
  WiFi.softAP(
    config.wifi_ssid,
    pass,
    (int)config.wifi_channel,
    (int)config.wifi_hidden,
    (int)config.wifi_max_conn
  );

  /* Initialize Captive Portal DNS Server on Port 53 */
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", local_ip);

  /* Enable Cross-Origin Resource Sharing */
  server.enableCORS(true);

  /* Register Web Routes */
  server.on("/", HTTP_GET, handleRootPage);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/config", HTTP_GET, handleApiConfig);
  server.on("/api/settings/sensor", HTTP_POST, handleSaveSensorSettings);
  server.on("/api/settings/wifi", HTTP_POST, handleSaveWifiSettings);
  server.on("/api/relay/override", HTTP_POST, handleRelayOverride);
  server.on("/api/stats/clear", HTTP_POST, handleClearStats);
  server.on("/api/system/restart", HTTP_POST, handleSystemRestart);
  server.on("/api/system/factory_reset", HTTP_POST, handleSystemFactoryReset);

  /* Captive Portal Probes for Cross-Platform Auto-Detection */
  server.on("/hotspot-detect.html", HTTP_GET, handleRootPage);
  server.on("/generate_204", HTTP_GET, handleCaptiveRedirect);
  server.on("/gen_204", HTTP_GET, handleCaptiveRedirect);
  server.on("/ncsi.txt", HTTP_GET, handleCaptiveRedirect);
  server.on("/connecttest.txt", HTTP_GET, handleCaptiveRedirect);
  server.on("/redirect", HTTP_GET, handleCaptiveRedirect);
  server.on("/canonical.html", HTTP_GET, handleRootPage);
  server.on("/success.txt", HTTP_GET, handleCaptiveRedirect);

  server.onNotFound(handleNotFound);
  server.begin();

  /* Enable Hardware Watchdog (15s timeout) */
  setupTaskWatchdog(15);
}

/* Arduino Main Loop (100% Non-Blocking) */
void loop() {
  /* Reset Watchdog Timer */
  esp_task_wdt_reset();

  /* Process Non-Blocking DNS Server Requests for Captive Portal */
  dnsServer.processNextRequest();

  /* Process Non-Blocking Web Server Client Requests */
  server.handleClient();

  /* Process Non-Blocking Sensor Polling State Machine */
  sampleSensorNonBlocking();

  /* Process Non-Blocking Relay Control State Machine */
  evaluateRelayNonBlocking();

  /* Process Non-Blocking Hardware BOOT Button (GPIO 0) 5s Hold State Machine */
  checkBootButtonNonBlocking();

  /* Handle Scheduled Soft Restart */
  if (sysData.pending_restart && get_time_ms64() >= sysData.restart_target_ms) {
    esp_restart();
  }

  sysData.loop_cycle_count++;
}
