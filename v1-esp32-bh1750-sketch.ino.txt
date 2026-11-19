// ESP32 BH1750 Controller

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <BH1750.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

// ---------------- Pin map ----------------
static constexpr uint8_t  PIN_I2C_SDA  = 21;
static constexpr uint8_t  PIN_I2C_SCL  = 22;
static constexpr uint8_t  PIN_RELAY    = 25;
static constexpr uint8_t  PIN_ALARM    = 26;
static constexpr uint8_t  PIN_BOOT     = 0;

// ---------------- Timing ----------------
static constexpr uint32_t SENSOR_INTERVAL_MS   = 200;
static constexpr uint32_t NVS_SAVE_DEBOUNCE_MS = 15000;
static constexpr uint32_t NVS_MIN_WRITE_GAP_MS = 120000;
static constexpr uint32_t FACTORY_HOLD_MS      = 5000;
static constexpr uint32_t WDT_TIMEOUT_S        = 30;
static constexpr uint32_t SENSOR_STALE_MS      = 10000;
static constexpr uint32_t SENSOR_LOST_FAIL_MS  = 60000;
static constexpr uint32_t BOOT_SANITY_MS       = 10000;
static constexpr uint32_t MANUAL_OVERRIDE_MS   = 60000;

// ---------------- Config model (CRC-protected) ----------------
struct __attribute__((packed)) Config {
    uint32_t magic;
    uint16_t version;
    uint16_t crc;

    uint32_t luxLow;
    uint32_t luxHigh;
    uint32_t alarmLux;
    uint32_t alarmClear;

    uint8_t  relayEnabled;
    uint8_t  alarmEnabled;
    uint8_t  invertRelay;
    uint8_t  reserved0;

    char     ssid[33];
    char     pass[65];

    uint32_t reserved1;
};

static constexpr uint32_t CONFIG_MAGIC   = 0xB8B5A10Eu;
static constexpr uint16_t CONFIG_VERSION = 1;

// ---------------- Globals ----------------
static Preferences        gPrefs;
static Config             gCfg;
static SemaphoreHandle_t  gCfgMutex = nullptr;

static WebServer          gHttp(80);
static DNSServer          gDns;
static BH1750             gLux;

static volatile bool      gWifiUp       = false;
static volatile bool      gCfgDirty     = false;
static volatile uint32_t  gLastSaveMs   = 0;
static volatile uint32_t  gLastDirtyMs  = 0;
static volatile bool      gFactoryReset = false;

static volatile uint32_t  gLuxRaw     = 0;
static volatile bool      gLuxValid   = false;
static volatile bool      gRelayState = false;
static volatile bool      gAlarmState = false;

static char     gJsonBuf[256];
static uint32_t gLastPoll32 = 0;
static uint64_t gUptimeMs = 0;
static uint32_t gLastUptime32 = 0;

static esp_timer_handle_t gRebootTimer = nullptr;
static uint32_t           gLastLuxUpdate32 = 0;
static bool               gLuxEverUpdated  = false;
static bool               gFirstMeasDone   = false;

static uint32_t gManualOverrideStart  = 0;
static bool     gManualOverrideActive = false;
static volatile bool gManualRelayTarget = false;

static uint32_t gSensorLostStart = 0;
static bool     gSensorLost      = false;

static uint32_t gBootSanityT0    = 0;
static bool     gBootSanityDone  = false;
static bool     gBootSanityArmed = false;

// ---------------- Forward declarations ----------------
static String extractNumber(const String& body, const char* key);
static String extractString(const String& body, const char* key);
static bool   parseU32(const String& s, uint32_t& out);
static bool   parseU8 (const String& s, uint8_t&  out);
static void   rebootTimerCb(void*);
static bool   failSafeActive();

// ---------------- CRC32 ----------------
static uint32_t crc32(const uint8_t* d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        c ^= d[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static void cfgCrc(Config& c) {
    c.crc = 0;
    c.crc = (uint16_t)(crc32(reinterpret_cast<const uint8_t*>(&c), sizeof(Config)) & 0xFFFFu);
}

static bool cfgValid(const Config& c) {
    if (c.magic   != CONFIG_MAGIC)   return false;
    if (c.version != CONFIG_VERSION) return false;
    Config t = c; cfgCrc(t);
    return t.crc == c.crc;
}

static void cfgDefaults(Config& c) {
    memset(&c, 0, sizeof(c));
    c.magic        = CONFIG_MAGIC;
    c.version      = CONFIG_VERSION;
    c.luxLow       = 50;
    c.luxHigh      = 100;
    c.alarmLux     = 20000;
    c.alarmClear   = 15000;
    c.relayEnabled = 1;
    c.alarmEnabled = 1;
    c.invertRelay  = 0;
    strncpy(c.ssid, "ESP32_BH1750", sizeof(c.ssid) - 1);
    strncpy(c.pass, "12345678",    sizeof(c.pass) - 1);
    cfgCrc(c);
}

static void cfgSanitize(Config& c) {
    if (c.luxLow  > 99999u)  c.luxLow  = 99999u;
    if (c.luxHigh > 100000u) c.luxHigh = 100000u;
    if (c.luxHigh <= c.luxLow) c.luxHigh = c.luxLow + 1u;

    if (c.alarmLux < 2u)      c.alarmLux = 2u;
    if (c.alarmLux > 100000u) c.alarmLux = 100000u;
    if (c.alarmClear >= c.alarmLux) c.alarmClear = c.alarmLux - 1u;

    c.ssid[sizeof(c.ssid) - 1] = 0;
    c.pass[sizeof(c.pass) - 1] = 0;
    size_t sl = strnlen(c.ssid, sizeof(c.ssid));
    if (sl < 1 || sl > 32) strncpy(c.ssid, "ESP32_BH1750", sizeof(c.ssid) - 1);
    size_t pl = strnlen(c.pass, sizeof(c.pass));
    if (pl < 8) strncpy(c.pass, "12345678", sizeof(c.pass) - 1);
}

// ---------------- NVS ----------------
static bool nvsLoad() {
    if (!gPrefs.begin("bhcfg", true)) return false;
    bool ok = false;
    if (gPrefs.getBytesLength("cfg") == sizeof(Config)) {
        Config t;
        if (gPrefs.getBytes("cfg", &t, sizeof(t)) == sizeof(t) && cfgValid(t)) {
            gCfg = t; ok = true;
        }
    }
    gPrefs.end();
    return ok;
}

static bool nvsSave() {
    Config t = gCfg; cfgSanitize(t); cfgCrc(t);
    if (!gPrefs.begin("bhcfg", false)) return false;
    bool ok = gPrefs.putBytes("cfg", &t, sizeof(t)) == sizeof(t);
    gPrefs.end();
    if (ok) { gCfg = t; gLastSaveMs = (uint32_t)millis(); }
    return ok;
}

static void cfgMarkDirty() {
    gCfgDirty    = true;
    gLastDirtyMs = (uint32_t)millis();
}

static void nvsMaybeFlush() {
    if (!gCfgDirty) return;
    if (xSemaphoreTake(gCfgMutex, 0) != pdTRUE) return;

    uint32_t now = (uint32_t)millis();
    if (now - gLastDirtyMs >= NVS_SAVE_DEBOUNCE_MS &&
        now - gLastSaveMs  >= NVS_MIN_WRITE_GAP_MS) {
        if (nvsSave()) gCfgDirty = false;
    }
    xSemaphoreGive(gCfgMutex);
}

// ---------------- GPIO ----------------
static void gpioInit() {
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_ALARM, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_ALARM, LOW);
    pinMode(PIN_BOOT, INPUT_PULLUP);
}

static void applyRelay() {
    bool want = gRelayState;
    if (gCfg.invertRelay) want = !want;
    digitalWrite(PIN_RELAY, want ? HIGH : LOW);
}

static void applyAlarm() { digitalWrite(PIN_ALARM, gAlarmState ? HIGH : LOW); }

// ---------------- Sensor ----------------
static void sensorInit() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
    gLuxValid = gLux.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
    if (gLuxValid) {
        gLastLuxUpdate32 = (uint32_t)millis();
        gLuxEverUpdated  = true;
    } else {
        gLuxEverUpdated = false;
    }
}

static void sensorPoll() {
    uint32_t now32 = (uint32_t)millis();

    if (!gLuxValid) {
        static uint32_t lastTry = 0;
        if ((uint32_t)(now32 - lastTry) > 1000) {
            lastTry = now32;
            gLuxValid = gLux.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
            if (gLuxValid) {
                gLastLuxUpdate32 = now32;
                gLuxEverUpdated  = true;
            }
        }
        return;
    }

    if (gLux.measurementReady()) {
        float f = gLux.readLightLevel();
        if (f < 0.0f) {
        } else {
            if (f > 100000.0f) f = 100000.0f;
            gLuxRaw          = (uint32_t)f;
            gLastLuxUpdate32 = now32;
            gLuxEverUpdated  = true;
            gFirstMeasDone   = true;
        }
    }
    if (gLuxValid && gLuxEverUpdated &&
        (uint32_t)(now32 - gLastLuxUpdate32) > SENSOR_STALE_MS) {
        gLuxValid       = false;
        gLuxEverUpdated = false;
    }
}

// ---------------- Control logic ----------------
static void controlTick() {
    if (!gFirstMeasDone) { applyRelay(); applyAlarm(); return; }
    uint32_t lux   = gLuxRaw;
    uint32_t now32 = (uint32_t)millis();

    if (gManualOverrideActive &&
        (uint32_t)(now32 - gManualOverrideStart) >= MANUAL_OVERRIDE_MS) {
        gManualOverrideActive = false;
    }

    if (!gCfg.relayEnabled) {
        gRelayState = false;
        gManualOverrideActive = false;
    } else if (gManualOverrideActive) {
        gRelayState = gManualRelayTarget;
    } else {
        if (!gRelayState && lux <  gCfg.luxLow)      gRelayState = true;
        else if (gRelayState && lux >= gCfg.luxHigh) gRelayState = false;
    }

    if (gCfg.alarmEnabled) {
        if (!gAlarmState && lux >  gCfg.alarmLux)        gAlarmState = true;
        else if (gAlarmState && lux <= gCfg.alarmClear)  gAlarmState = false;
    } else gAlarmState = false;

    if (!gLuxValid) {
        if (!gSensorLost) { gSensorLost = true; gSensorLostStart = now32; }
        if ((uint32_t)(now32 - gSensorLostStart) >= SENSOR_LOST_FAIL_MS) {
            gRelayState = false;
            if (gCfg.alarmEnabled) gAlarmState = true;
        }
    } else {
        gSensorLost = false;
    }

    applyRelay();
    applyAlarm();
}

static bool failSafeActive() {
    return !gLuxValid && gSensorLost &&
           ((uint32_t)millis() - gSensorLostStart) >= SENSOR_LOST_FAIL_MS;
}

// ---------------- BOOT factory reset ----------------
static void bootButtonTick() {
    static uint32_t pressedAt  = 0;
    static bool     wasPressed = false;
    uint32_t now32 = (uint32_t)millis();

    if (!gBootSanityDone) {
        if (!gBootSanityArmed) { gBootSanityT0 = now32; gBootSanityArmed = true; }
        if ((uint32_t)(now32 - gBootSanityT0) >= BOOT_SANITY_MS) gBootSanityDone = true;
        return;
    }

    bool pressed = (digitalRead(PIN_BOOT) == LOW);
    if (pressed && !wasPressed) { pressedAt = now32; wasPressed = true; }
    else if (!pressed)          { wasPressed = false; }
    else if (wasPressed && (uint32_t)(now32 - pressedAt) >= FACTORY_HOLD_MS) gFactoryReset = true;
}

static void doFactoryReset() {
    gPrefs.begin("bhcfg", false);
    gPrefs.clear();
    gPrefs.end();
    WiFi.disconnect(true, true);
    esp_restart();
}

// ---------------- WiFi AP + captive portal ----------------
static void wifiStart() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAPdisconnect(true);
    WiFi.softAP(gCfg.ssid, gCfg.pass, 6, 0, 4);
    IPAddress ap = WiFi.softAPIP();
    gDns.setErrorReplyCode(DNSReplyCode::NoError);
    gDns.start(53, "*", ap);
    gWifiUp = true;
}

static void captiveTick() {
    if (gWifiUp) gDns.processNextRequest();
}

// ---------------- PROGMEM UI ----------------
static const char P_HTML_HEAD[] PROGMEM =
"<!doctype html><html lang='en'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 BH1750</title><style>"
":root{--b:#0b6;--d:#043;--t:#111}"
"*{box-sizing:border-box}body{margin:0;font:16px system-ui,Segoe UI,Arial;background:#f5f7f6;color:var(--t)}"
"header{background:var(--d);color:#fff;padding:14px 18px;display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:space-between}"
"header h1{font-size:18px;margin:0}nav{display:flex;gap:8px;flex-wrap:wrap}"
"nav a{color:#fff;text-decoration:none;padding:6px 10px;border-radius:6px;background:#0002}"
"nav a.on{background:var(--b)}main{max-width:880px;margin:0 auto;padding:16px;display:grid;gap:14px}"
".card{background:#fff;border-radius:12px;padding:16px;box-shadow:0 1px 3px #0002}"
"label{display:block;margin:8px 0 4px;font-weight:600}"
"input,button,select{font:inherit;padding:12px;border-radius:8px;border:1px solid #bbb;width:100%}"
"button{background:var(--b);color:#fff;border:0;font-weight:700;cursor:pointer}"
"button:disabled{background:#999;cursor:not-allowed}button:active{filter:brightness(.9)}nav a{padding:10px 14px}"
"button.sec{background:#555}.row{display:grid;gap:10px;grid-template-columns:1fr 1fr}"
".stat{display:grid;gap:6px;grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}"
".stat div{background:#eef;padding:10px;border-radius:8px}"
".stat b{display:block;font-size:22px;color:var(--d);font-variant-numeric:tabular-nums;overflow-wrap:anywhere}"
".led{display:inline-block;width:12px;height:12px;border-radius:50%;background:#c00;margin-right:6px;vertical-align:middle}"
".led.on{background:#0c0}@media(max-width:520px){.row{grid-template-columns:1fr}}"
"</style></head><body>";

static const char P_HTML_NAV_DASH[] PROGMEM =
"<header><h1>ESP32 BH1750</h1><nav>"
"<a class='on' href='/'>Dashboard</a>"
"<a href='/settings'>Sensor Settings</a>"
"<a href='/wifi'>WiFi / AP</a>"
"<a href='/reset'>Reset</a>"
"</nav></header><main>";

static const char P_HTML_NAV_SET[] PROGMEM =
"<header><h1>ESP32 BH1750</h1><nav>"
"<a href='/'>Dashboard</a>"
"<a class='on' href='/settings'>Sensor Settings</a>"
"<a href='/wifi'>WiFi / AP</a>"
"<a href='/reset'>Reset</a>"
"</nav></header><main>";

static const char P_HTML_NAV_WIFI[] PROGMEM =
"<header><h1>ESP32 BH1750</h1><nav>"
"<a href='/'>Dashboard</a>"
"<a href='/settings'>Sensor Settings</a>"
"<a class='on' href='/wifi'>WiFi / AP</a>"
"<a href='/reset'>Reset</a>"
"</nav></header><main>";

static const char P_HTML_NAV_RESET[] PROGMEM =
"<header><h1>ESP32 BH1750</h1><nav>"
"<a href='/'>Dashboard</a>"
"<a href='/settings'>Sensor Settings</a>"
"<a href='/wifi'>WiFi / AP</a>"
"<a class='on' href='/reset'>Reset</a>"
"</nav></header><main>";

static const char P_HTML_TAIL[] PROGMEM = "</main></body></html>";

static const char P_HTML_DASH[] PROGMEM =
"<section class='card'><h2>Live Status</h2>"
"<div class='stat'>"
"<div><span>Lux</span><b id='lux' aria-live='polite'>--</b></div>"
"<div><span>Relay</span><b><span class='led' id='rled'></span><span id='relay' aria-live='polite'>--</span></b></div>"
"<div><span>Alarm</span><b><span class='led' id='aled'></span><span id='alarm' aria-live='polite'>--</span></b></div>"
"<div><span>Uptime (s)</span><b id='up' aria-live='off'>--</b></div>"
"<div><span>Sensor</span><b id='sv' aria-live='polite'>--</b></div>"
"</div></section>"
"<section class='card'><h2>Quick Actions</h2>"
"<div class='row'>"
"<button id='btnRefresh'>Refresh</button>"
"<button class='sec' id='btnRelay' disabled>Toggle Relay</button>"
"</div></section>"
"<script>"
"const $=id=>document.getElementById(id);"
"async function tick(){try{const c=new AbortController();const t=setTimeout(()=>c.abort(),3000);const r=await fetch('/api/status',{cache:'no-store',signal:c.signal});clearTimeout(t);if(!r.ok)throw 0;const j=await r.json();"
"$('lux').textContent=j.lux;$('relay').textContent=j.relay?'ON':'OFF';"
"$('rled').className='led '+(j.relay?'on':'');"
"$('alarm').textContent=j.alarm?'ON':'OFF';"
"$('aled').className='led '+(j.alarm?'on':'');"
"$('up').textContent=j.uptime;$('sv').textContent=j.sensor?'OK':'ERR';"
"$('btnRelay').disabled=(j.relayEnabled===0)||(j.failSafe===true);"
"}catch(e){$('sv').textContent='LINK';$('btnRelay').disabled=true;}}"
"async function toggleRelay(){await fetch('/api/relay',{method:'POST'});await tick();}"
"$('btnRefresh').onclick=tick;$('btnRelay').onclick=toggleRelay;"
"tick();setInterval(tick,1000);document.addEventListener('visibilitychange',()=>{if(!document.hidden)tick();});"
"</script><noscript><p>Enable JavaScript for live status.</p></noscript>";

static const char P_HTML_SET[] PROGMEM =
"<section class='card'><h2>Sensor &amp; Thresholds</h2><form id='f'>"
"<div class='row'>"
"<div><label>Relay ON below (lux)</label><input name='luxLow' type='number' inputmode='numeric' min='0' step='1'></div>"
"<div><label>Relay OFF at/above (lux)</label><input name='luxHigh' type='number' inputmode='numeric' min='0' step='1'></div>"
"</div>"
"<div class='row'>"
"<div><label>Alarm ON above (lux)</label><input name='alarmLux' type='number' inputmode='numeric' min='0' step='1'></div>"
"<div><label>Alarm OFF below (lux)</label><input name='alarmClear' type='number' inputmode='numeric' min='0' step='1'></div>"
"</div>"
"<div class='row'>"
"<div><label>Relay enabled</label><select name='relayEnabled'><option value='1'>Yes</option><option value='0'>No</option></select></div>"
"<div><label>Alarm enabled</label><select name='alarmEnabled'><option value='1'>Yes</option><option value='0'>No</option></select></div>"
"</div>"
"<div class='row'>"
"<div><label>Invert relay</label><select name='invertRelay'><option value='0'>No</option><option value='1'>Yes</option></select></div>"
"<div><label>Sensor status</label><input id='sv' disabled></div>"
"</div>"
"<button type='submit'>Save Settings</button>"
"</form></section>"
"<script>"
"const $=id=>document.getElementById(id);"
"async function load(){try{const r=await fetch('/api/config',{cache:'no-store'});if(!r.ok)throw 0;const j=await r.json();const f=$('f');"
"for(const k in j){if(f[k]!==undefined)f[k].value=j[k];}$('sv').value=j.sensor?'OK':'ERR';}catch(e){$('sv').value='LINK';}}"
"$('f').onsubmit=async e=>{e.preventDefault();const fd=new FormData(e.target);const o={};"
"fd.forEach((v,k)=>o[k]=v);const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});"
"if(r.ok){alert('Saved');load();}else if(r.status===400){alert('Invalid value - check all numeric fields (no blanks, no negatives)');}else if(r.status===503){alert('Busy, retry');}else{alert('Error '+r.status);}};load();"
"</script>";

static const char P_HTML_WIFI[] PROGMEM =
"<section class='card'><h2>Access Point</h2><form id='f'>"
"<label>SSID (1-32)</label><input name='ssid' maxlength='32' required>"
"<label>Password (8-63)</label><input name='pass' type='password' maxlength='63' minlength='8' required>"
"<div class='row'>"
"<div><label>SSID current</label><input id='cs' disabled></div>"
"<div><label>Clients</label><input id='cc' disabled></div>"
"</div>"
"<button type='submit'>Apply &amp; Reboot</button>"
"</form></section>"
"<script>"
"const $=id=>document.getElementById(id);"
"async function load(){try{const r=await fetch('/api/wifi',{cache:'no-store'});if(!r.ok)throw 0;const j=await r.json();"
"$('f').ssid.value=j.ssid;$('cs').value=j.ssid;$('cc').value=j.clients;}catch(e){$('cs').value='LINK';$('cc').value='';}}"
"$('f').onsubmit=async e=>{e.preventDefault();const fd=new FormData(e.target);const o={};"
"fd.forEach((v,k)=>o[k]=v);try{const r=await fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});"
"if(r.ok){const j=await r.json();alert(j.reboot===false?'Saved (no reboot)':'Saved. Rebooting...');}else if(r.status===400){alert('Invalid SSID or password');$('f').pass.value='';}else{alert('Save failed '+r.status);}load();}catch(e){alert('Network error - try again');}};load();"
"</script>";

static const char P_HTML_RESET[] PROGMEM =
"<section class='card'><h2>Factory Reset</h2>"
"<p>Hold <b>BOOT (GPIO0)</b> for 5 seconds to factory-reset. Or confirm below.</p>"
"<button class='sec' id='btn'>Confirm Factory Reset</button></section>"
"<script>const b=document.getElementById('btn');b.onclick=async()=>{if(!confirm('Erase all settings?'))return;"
"try{const r=await fetch('/api/reset',{method:'POST'});if(!r.ok){alert('Reset failed '+r.status);return;}alert('Resetting...');}catch(e){alert('Network error - reset not confirmed');}};</script>";

// ---------------- HTML send helpers ----------------
static void sendPage(const char* nav, const char* body) {
    gHttp.setContentLength(CONTENT_LENGTH_UNKNOWN);
    gHttp.send(200, "text/html; charset=utf-8", "");
    gHttp.sendContent_P(P_HTML_HEAD);
    gHttp.sendContent_P(nav);
    gHttp.sendContent_P(body);
    gHttp.sendContent_P(P_HTML_TAIL);
    gHttp.sendContent("");
}

// ---------------- JSON field parsers ----------------
static String extractNumber(const String& body, const char* key) {
    int i = body.indexOf(key);
    if (i < 0) return String();
    i = body.indexOf(':', i);
    if (i < 0) return String();
    int j = i + 1;
    while (j < (int)body.length() && (body[j] == ' ' || body[j] == '"')) j++;
    int k = j;
    while (k < (int)body.length() && body[k] != ',' && body[k] != '}' && body[k] != '"') k++;
    return body.substring(j, k);
}

static String extractString(const String& body, const char* key) {
    int i = body.indexOf(key);
    if (i < 0) return String();
    i = body.indexOf(':', i);
    if (i < 0) return String();
    i = body.indexOf('"', i);
    if (i < 0) return String();
    int j = body.indexOf('"', i + 1);
    if (j < 0) return String();
    return body.substring(i + 1, j);
}

static bool parseU32(const String& s, uint32_t& out) {
    if (s.length() == 0 || s.length() > 10) return false;
    if (s[0] == '-') return false;
    char* e = nullptr;
    unsigned long v = strtoul(s.c_str(), &e, 10);
    if (!e || *e) return false;
    out = (uint32_t)v;
    return true;
}

static bool parseU8(const String& s, uint8_t& out) {
    uint32_t v;
    if (!parseU32(s, v)) return false;
    if (v > 1) return false;
    out = (uint8_t)v;
    return true;
}

// ---------------- HTTP handlers ----------------
static void handleRoot()     { sendPage(P_HTML_NAV_DASH,  P_HTML_DASH);  }
static void handleSettings() { sendPage(P_HTML_NAV_SET,   P_HTML_SET);   }
static void handleWifiPage() { sendPage(P_HTML_NAV_WIFI,  P_HTML_WIFI);  }
static void handleResetPage(){ sendPage(P_HTML_NAV_RESET, P_HTML_RESET); }

static void handleStatus() {
    int n = snprintf(gJsonBuf, sizeof(gJsonBuf),
        "{\"lux\":%lu,\"relay\":%s,\"alarm\":%s,\"uptime\":%llu,"
        "\"sensor\":%s,\"relayEnabled\":%u,\"failSafe\":%s}",
        (unsigned long)gLuxRaw,
        gRelayState ? "true" : "false",
        gAlarmState ? "true" : "false",
        (unsigned long long)(gUptimeMs / 1000ULL),
        gLuxValid ? "true" : "false",
        (unsigned)gCfg.relayEnabled,
        failSafeActive() ? "true" : "false");
    if (n < 0 || n >= (int)sizeof(gJsonBuf)) {
        gHttp.send(500, "application/json", F("{\"e\":1}"));
        return;
    }
    gHttp.setContentLength((size_t)n);
    gHttp.send(200, "application/json", "");
    gHttp.sendContent(gJsonBuf, n);
}

static void handleRelayToggle() {
    if (!gCfg.relayEnabled) {
        gHttp.send(409, "application/json", F("{\"e\":10}"));
        return;
    }
    if (failSafeActive()) {
        gHttp.send(409, "application/json", F("{\"e\":11}"));
        return;
    }
    gManualRelayTarget    = !gRelayState;
    gRelayState           = gManualRelayTarget;
    gManualOverrideStart  = (uint32_t)millis();
    gManualOverrideActive = true;
    applyRelay();
    gHttp.send(200, "application/json", F("{\"ok\":true}"));
}

static void handleConfigGet() {
    int n = snprintf(gJsonBuf, sizeof(gJsonBuf),
        "{\"luxLow\":%lu,\"luxHigh\":%lu,\"alarmLux\":%lu,\"alarmClear\":%lu,"
        "\"relayEnabled\":%u,\"alarmEnabled\":%u,\"invertRelay\":%u,\"sensor\":%s}",
        (unsigned long)gCfg.luxLow, (unsigned long)gCfg.luxHigh,
        (unsigned long)gCfg.alarmLux, (unsigned long)gCfg.alarmClear,
        (unsigned)gCfg.relayEnabled, (unsigned)gCfg.alarmEnabled,
        (unsigned)gCfg.invertRelay, gLuxValid ? "true" : "false");
    if (n < 0 || n >= (int)sizeof(gJsonBuf)) {
        gHttp.send(500, "application/json", F("{\"e\":1}"));
        return;
    }
    gHttp.setContentLength((size_t)n);
    gHttp.send(200, "application/json", "");
    gHttp.sendContent(gJsonBuf, n);
}

static void handleConfigPost() {
    if (!gHttp.hasArg("plain")) { gHttp.send(400, "application/json", F("{\"e\":1}")); return; }
    if ((int)gHttp.clientContentLength() > 2048) {
        gHttp.send(413, "application/json", F("{\"e\":13}")); return;
    }
    String body = gHttp.arg("plain");
    if (body.length() > 2048) { gHttp.send(413, "application/json", F("{\"e\":13}")); return; }

    uint32_t ul = 0, uh = 0, ua = 0, uac = 0;
    uint8_t  re = 0, ae = 0, ir = 0;
    bool ok =
        parseU32(extractNumber(body, "\"luxLow\""),      ul)  &&
        parseU32(extractNumber(body, "\"luxHigh\""),     uh)  &&
        parseU32(extractNumber(body, "\"alarmLux\""),    ua)  &&
        parseU32(extractNumber(body, "\"alarmClear\""),  uac) &&
        parseU8 (extractNumber(body, "\"relayEnabled\""),re)  &&
        parseU8 (extractNumber(body, "\"alarmEnabled\""),ae)  &&
        parseU8 (extractNumber(body, "\"invertRelay\""), ir);
    if (!ok) { gHttp.send(400, "application/json", F("{\"e\":2}")); return; }

    if (xSemaphoreTake(gCfgMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        gHttp.send(503, "application/json", F("{\"e\":12}"));
        return;
    }
    gCfg.luxLow       = ul;
    gCfg.luxHigh      = uh;
    gCfg.alarmLux     = ua;
    gCfg.alarmClear   = uac;
    gCfg.relayEnabled = re;
    gCfg.alarmEnabled = ae;
    gCfg.invertRelay  = ir;
    cfgSanitize(gCfg);
    cfgMarkDirty();
    xSemaphoreGive(gCfgMutex);
    gHttp.send(200, "application/json", F("{\"ok\":true}"));
}

static void handleWifiGet() {
    int n = snprintf(gJsonBuf, sizeof(gJsonBuf),
        "{\"ssid\":\"%s\",\"clients\":%u}",
        gCfg.ssid, (unsigned)WiFi.softAPgetStationNum());
    if (n < 0 || n >= (int)sizeof(gJsonBuf)) {
        gHttp.send(500, "application/json", F("{\"e\":1}"));
        return;
    }
    gHttp.setContentLength((size_t)n);
    gHttp.send(200, "application/json", "");
    gHttp.sendContent(gJsonBuf, n);
}

static void rebootTimerCb(void*) { esp_restart(); }

static void handleWifiPost() {
    if (!gHttp.hasArg("plain")) { gHttp.send(400, "application/json", F("{\"e\":1}")); return; }
    if ((int)gHttp.clientContentLength() > 2048) {
        gHttp.send(413, "application/json", F("{\"e\":13}")); return;
    }
    String body = gHttp.arg("plain");
    if (body.length() > 2048) { gHttp.send(413, "application/json", F("{\"e\":13}")); return; }

    String ssid = extractString(body, "\"ssid\"");
    String pass = extractString(body, "\"pass\"");
    if (ssid.length() < 1 || ssid.length() > 32)  { gHttp.send(400, "application/json", F("{\"e\":2}")); return; }
    if (pass.length() < 8 || pass.length() > 63)  { gHttp.send(400, "application/json", F("{\"e\":3}")); return; }
    for (size_t i = 0; i < ssid.length(); ++i) {
        uint8_t c = (uint8_t)ssid[i];
        if (c < 0x20 || c == 0x7F) { gHttp.send(400, "application/json", F("{\"e\":4}")); return; }
    }
    for (size_t i = 0; i < pass.length(); ++i) {
        uint8_t c = (uint8_t)pass[i];
        if (c < 0x20 || c == 0x7F) { gHttp.send(400, "application/json", F("{\"e\":5}")); return; }
    }
    if (ssid.indexOf('"') >= 0 || ssid.indexOf('\\') >= 0) { gHttp.send(400, "application/json", F("{\"e\":4}")); return; }
    if (pass.indexOf('"') >= 0 || pass.indexOf('\\') >= 0) { gHttp.send(400, "application/json", F("{\"e\":5}")); return; }

    bool ok = false;
    if (xSemaphoreTake(gCfgMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        memset(gCfg.ssid, 0, sizeof(gCfg.ssid));
        memset(gCfg.pass, 0, sizeof(gCfg.pass));
        memcpy(gCfg.ssid, ssid.c_str(), ssid.length());
        memcpy(gCfg.pass, pass.c_str(), pass.length());
        cfgSanitize(gCfg);
        ok = nvsSave();
        if (ok) gCfgDirty = false;
        xSemaphoreGive(gCfgMutex);
    }
    if (!ok) { gHttp.send(500, "application/json", F("{\"e\":9}")); return; }

    bool rebooting = false;
    if (!gRebootTimer) {
        esp_timer_create_args_t a = {};
        a.callback = rebootTimerCb;
        a.name     = "reboot";
        if (esp_timer_create(&a, &gRebootTimer) != ESP_OK) gRebootTimer = nullptr;
    }
    if (gRebootTimer) {
        if (esp_timer_is_active(gRebootTimer)) {
            rebooting = true;
        } else {
            rebooting = (esp_timer_start_once(gRebootTimer, 1500000ULL) == ESP_OK);
        }
    }

    gHttp.send(200, "application/json",
               rebooting ? F("{\"ok\":true}") : F("{\"ok\":true,\"reboot\":false}"));
    gHttp.client().flush();
}

static void handleResetPost() {
    gHttp.send(200, "application/json", F("{\"ok\":true}"));
    gHttp.client().flush();
    delay(300);  
    gFactoryReset = true;
}

// ---------------- Setup ----------------
void setup() {
    gpioInit();

    gCfgMutex = xSemaphoreCreateMutex();
    if (!gCfgMutex) { esp_restart(); }

    if (!nvsLoad()) { cfgDefaults(gCfg); nvsSave(); }
    cfgSanitize(gCfg);

    gLastSaveMs = (uint32_t)millis() - NVS_MIN_WRITE_GAP_MS;

    sensorInit();
    wifiStart();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    {
        esp_task_wdt_config_t wdtc = {};
        wdtc.timeout_ms     = (uint32_t)WDT_TIMEOUT_S * 1000U;
        wdtc.idle_core_mask = 0;
        wdtc.trigger_panic  = true;
        esp_task_wdt_init(&wdtc);
    }
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    esp_task_wdt_add(NULL);

    gHttp.onNotFound([]() {
        String uri = gHttp.uri();
        if (uri == "/hotspot-detect.html" || uri == "/library/test/success.html" ||
            uri == "/generate_204"        || uri == "/gen_204" ||
            uri == "/connecttest.txt"     || uri == "/ncsi.txt" ||
            uri == "/success.txt"         || uri == "/redirect") {
            gHttp.send(200, "text/html",
                F("<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"));
            return;
        }
        String host = gHttp.hostHeader();
        int colon = host.indexOf(':');
        if (colon >= 0) host = host.substring(0, colon);
        if (host != WiFi.softAPIP().toString()) {
            gHttp.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
            gHttp.send(302, "text/plain", "");
        } else {
            gHttp.send(404, "text/plain", "Not found");
        }
    });

    gHttp.on("/",           HTTP_GET,  handleRoot);
    gHttp.on("/settings",   HTTP_GET,  handleSettings);
    gHttp.on("/wifi",       HTTP_GET,  handleWifiPage);
    gHttp.on("/reset",      HTTP_GET,  handleResetPage);
    gHttp.on("/api/status", HTTP_GET,  handleStatus);
    gHttp.on("/api/relay",  HTTP_POST, handleRelayToggle);
    gHttp.on("/api/config", HTTP_GET,  handleConfigGet);
    gHttp.on("/api/config", HTTP_POST, handleConfigPost);
    gHttp.on("/api/wifi",   HTTP_GET,  handleWifiGet);
    gHttp.on("/api/wifi",   HTTP_POST, handleWifiPost);
    gHttp.on("/api/reset",  HTTP_POST, handleResetPost);
    gHttp.begin();

    gLastPoll32   = (uint32_t)millis();
    gLastUptime32 = gLastPoll32;
}

// ---------------- Loop ----------------
void loop() {
    esp_task_wdt_reset();

    uint32_t now32 = (uint32_t)millis();
    gUptimeMs += (uint32_t)(now32 - gLastUptime32);
    gLastUptime32 = now32;

    if ((uint32_t)(now32 - gLastPoll32) >= SENSOR_INTERVAL_MS) {
        gLastPoll32 = now32;
        sensorPoll();
        controlTick();
    }

    bootButtonTick();
    if (gFactoryReset) { gFactoryReset = false; doFactoryReset(); }

    gHttp.handleClient();
    captiveTick();
    nvsMaybeFlush();

    delay(1);
}
