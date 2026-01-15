#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ld2410.h>
#include <time.h>

#include "secrets.h"   // <- legger hemmeligheter her (SSID, URLer, TLS-policy)

// =========================
// SETTINGS
// =========================
#define DEBUG_NET   0
#define DEBUG_RADAR 0
#define DEBUG_TIME  0

IPAddress DNS1(1,1,1,1);
IPAddress DNS2(8,8,8,8);

// Backend port
static const uint16_t HTTPS_PORT = 443;

// Radar pins (din lodding)
#define RADAR_RX_PIN 33
#define RADAR_TX_PIN 18
#define RADAR_BAUD   256000

// Timing
static const uint32_t WIFI_RETRY_MS   = 3000;
static const uint32_t RADAR_EVAL_MS   = 25;      // 40 Hz eval
static const uint32_t COOLDOWN_MS     = 5000;    // 5 sek
static const uint32_t WAIT_CLEAR_MS   = 1800;    // kontinuerlig fravær før re-arm
static const uint32_t TRACK_WINDOW_MS = 1400;    // litt rausere vindu

// Trigger tuning (cm)
static const int ENTER_CM       = 340;
static const int PRE_ENTER_CM   = 550;  // prod-tunet for ~1.6s open-latens
static const int EXIT_CM        = 470;
static const int TRACK_START_CM = 650;

// Sensor hygiene
static const int MIN_VALID_CM = 55;
static const int MAX_VALID_CM = 650;
static const int JUMP_CM      = 160;

// Approach / hastighet
static const float V_MIN_MS  = 0.25f;
static const float V_FAST_MS = 0.38f; // 2 av 3 samples
static const float V_MAX_MS  = 3.00f;

// Score
static const int SCORE_TRIGGER = 8;
static const int SCORE_DECAY   = 1;

// Warmup
static const int WARMUP_ON_CM            = 950;
static const int WARMUP_OFF_CM           = 1200;
static const uint32_t WARMUP_COOLDOWN_MS = 6000;

// Debug webhook push
static const uint32_t DEBUG_PUSH_MS = 30000;

// NTP
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.nist.gov";
static const long  TZ_OFFSET_SEC = 3600;
static const int   DST_OFFSET_SEC = 0;

// Fornuftig cap i statistikk (unngår max=31 m/s ved glitch)
static const float V_STATS_CAP = 6.0f;

// Hvor lenge vi prøver å lese HTTP statuslinje i open-kallet (ms)
// (Du har vist at døra åpner selv om status ikke rekker å komme tilbake)
static const uint32_t STATUS_WAIT_MS = 450;

// =========================
// DEBUG MACROS
// =========================
#if DEBUG_NET || DEBUG_RADAR || DEBUG_TIME
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
  #define DBGFP(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGFP(...)
#endif

// =========================
// GLOBALS
// =========================
ld2410 radar;

enum State : uint8_t { ST_ARMED, ST_TRACKING, ST_COOLDOWN, ST_WAIT_CLEAR };
State st = ST_ARMED;

uint32_t lastWifiAttemptMs = 0;
uint32_t lastRadarEvalMs   = 0;

uint32_t trackSinceMs      = 0;
uint32_t cooldownSinceMs   = 0;
uint32_t clearSinceMs      = 0;

int score = 0;

// distance filter
int   lastRawCm   = 0;
float dFiltCm     = NAN;

// velocity tracking (normal, basert på filt)
float lastVms = 0.0f;

// fast-pass: mer responsivt filter + vFast
float dFastCm = NAN;
float lastVfast = 0.0f;

// fast-pass evidence (2 av siste 3)
bool fpRing[3] = {false, false, false};
uint8_t fpIdx = 0;

// warmup
bool warmupArmed = true;
uint32_t lastWarmupMs = 0;

// cached DNS
IPAddress cachedIP;
bool hasCachedIP = false;
uint32_t lastDnsMs = 0;

// NTP
bool ntpReady = false;

// Debug push
uint32_t lastDebugPushMs = 0;

// =========================
// DEBUG STATS SNAPSHOT
// =========================
struct DebugStats {
  uint32_t sinceMs = 0;

  uint32_t samples = 0;
  uint32_t presenceCount = 0;
  uint32_t hasRawCount = 0;

  int cmRawLast = 0;
  int cmFiltLast = 0;

  int cmRawMin = 99999;
  int cmRawMax = 0;

  float vLast = 0;
  float vFastLast = 0;
  float vMax = 0;

  int scoreLast = 0;
  int fpCntLast = 0;

  uint32_t triggers = 0;

  // Open tracking (send vs status)
  uint32_t openSent = 0;
  uint32_t openGotStatus = 0;
  uint32_t opensOk = 0;
  uint32_t opensFail = 0;
  int      codeLast = 0;

  uint32_t openMsLast = 0;
  uint32_t openMsMin = 999999;
  uint32_t openMsMax = 0;
  uint32_t openMsSum = 0;

  uint32_t warmups = 0;

  void reset(uint32_t now) {
    *this = DebugStats{};
    sinceMs = now;
  }

  void noteSample(bool presence, bool hasRaw, int cmRaw, int cmFilt,
                  float v, float vFast, int sc, int fpCnt) {
    samples++;
    if (presence) presenceCount++;
    if (hasRaw) hasRawCount++;

    cmRawLast = cmRaw;
    cmFiltLast = cmFilt;
    scoreLast = sc;
    fpCntLast = fpCnt;

    vLast = v;
    vFastLast = vFast;

    float av = fabs(v);
    float avf = fabs(vFast);
    if (av < V_STATS_CAP && av > vMax) vMax = av;
    if (avf < V_STATS_CAP && avf > vMax) vMax = avf;

    if (hasRaw && cmRaw > 0) {
      if (cmRaw < cmRawMin) cmRawMin = cmRaw;
      if (cmRaw > cmRawMax) cmRawMax = cmRaw;
    }
  }

  void noteTrigger() { triggers++; }
  void noteWarmup() { warmups++; }

  void noteOpenSent() { openSent++; }

  void noteOpenStatus(int code) {
    openGotStatus++;
    codeLast = code;
    if (code > 0 && code < 400) opensOk++;
    else if (code != 0) opensFail++; // code==0 betyr "ikke fått status"
  }

  void noteOpenTiming(uint32_t ms) {
    openMsLast = ms;
    if (ms < openMsMin) openMsMin = ms;
    if (ms > openMsMax) openMsMax = ms;
    openMsSum += ms;
  }

  uint32_t openMsAvg() const {
    uint32_t n = openSent;
    return (n == 0) ? 0 : (openMsSum / n);
  }
};

DebugStats dbg;

// =========================
// HELPERS
// =========================
const char* stateName(State s) {
  switch (s) {
    case ST_ARMED:      return "ARMED";
    case ST_TRACKING:   return "TRACK";
    case ST_COOLDOWN:   return "COOLDOWN";
    case ST_WAIT_CLEAR: return "WAIT_CLEAR";
    default:            return "?";
  }
}

void setState(State ns, uint32_t now) {
  (void)now;
  st = ns;

  if (ns == ST_TRACKING) {
    trackSinceMs = millis();
    score = 0;
    fpRing[0] = fpRing[1] = fpRing[2] = false;
    fpIdx = 0;
  }

  if (ns == ST_WAIT_CLEAR) {
    clearSinceMs = 0;
  }
}

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = now;

  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, DNS1, DNS2);
  WiFi.begin(WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(100);
}

bool initRadar() {
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(Serial1);

  uint32_t t0 = millis();
  bool sawFrame = false;
  while (millis() - t0 < 2000) {
    if (radar.read()) sawFrame = true;
    delay(5);
  }
  return sawFrame;
}

struct RadarMeas {
  bool presence;
  bool mDet;
  bool sDet;
  int  mCm;
  int  sCm;
  int  chosenCm;
  bool usedMoving;
};

static inline bool saneDist(int cm) {
  return (cm >= MIN_VALID_CM && cm <= MAX_VALID_CM);
}

int chooseDistancePreferred(bool mDet, int mCm, bool sDet, int sCm, bool &usedMoving) {
  usedMoving = false;
  if (mDet && saneDist(mCm)) { usedMoving = true;  return mCm; }
  if (sDet && saneDist(sCm)) { usedMoving = false; return sCm; }
  return 0;
}

RadarMeas readRadar(uint32_t now) {
  (void)now;
  RadarMeas out{};
  out.presence = radar.presenceDetected();

  out.mDet = radar.movingTargetDetected();
  out.mCm  = out.mDet ? radar.movingTargetDistance() : 0;

  out.sDet = radar.stationaryTargetDetected();
  out.sCm  = out.sDet ? radar.stationaryTargetDistance() : 0;

  out.usedMoving = false;
  out.chosenCm = 0;

  if (!out.presence) return out;

  int raw = chooseDistancePreferred(out.mDet, out.mCm, out.sDet, out.sCm, out.usedMoving);

  if (raw > 0 && lastRawCm > 0 && abs(raw - lastRawCm) > JUMP_CM) {
    raw = 0;
  }
  if (raw > 0) lastRawCm = raw;

  out.chosenCm = raw;
  return out;
}

void updateFilterAndVelocity(uint32_t now, int rawCm, bool hasRaw) {
  static float prevFilt = NAN;
  static uint32_t prevMs = 0;

  if (hasRaw) {
    if (!isfinite(dFiltCm)) dFiltCm = (float)rawCm;
    else {
      const float a = 0.55f;
      dFiltCm = a * (float)rawCm + (1.0f - a) * dFiltCm;
    }

    if (!isfinite(dFastCm)) dFastCm = (float)rawCm;
    else {
      const float af = 0.80f;
      dFastCm = af * (float)rawCm + (1.0f - af) * dFastCm;
    }
  }

  if (hasRaw && isfinite(dFiltCm) && isfinite(prevFilt) && prevMs != 0) {
    float dt = (now - prevMs) / 1000.0f;
    if (dt > 0.015f && dt < 0.6f) {
      lastVms = ((prevFilt - dFiltCm) / 100.0f) / dt;
    } else {
      lastVms = 0.0f;
    }
  } else {
    lastVms = 0.0f;
  }

  static float prevFast = NAN;
  static uint32_t prevFastMs = 0;
  if (hasRaw && isfinite(dFastCm) && isfinite(prevFast) && prevFastMs != 0) {
    float dt = (now - prevFastMs) / 1000.0f;
    if (dt > 0.015f && dt < 0.6f) {
      lastVfast = ((prevFast - dFastCm) / 100.0f) / dt;
    } else {
      lastVfast = 0.0f;
    }
  } else {
    lastVfast = 0.0f;
  }
  prevFast = dFastCm;
  prevFastMs = now;

  prevFilt = dFiltCm;
  prevMs = now;
}

int fastPassCount() {
  int c = 0;
  for (int i = 0; i < 3; i++) if (fpRing[i]) c++;
  return c;
}

void pushFastPassEvidence(bool ok) {
  fpRing[fpIdx] = ok;
  fpIdx = (fpIdx + 1) % 3;
}

bool refreshDnsCache(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (now - lastDnsMs < 10000) return hasCachedIP;
  lastDnsMs = now;

  IPAddress ip;
  bool ok = (WiFi.hostByName(HOST, ip) == 1);
  if (ok) {
    cachedIP = ip;
    hasCachedIP = true;
  }
  return ok;
}

bool warmupHttpsHeadRoot(uint32_t now) {
  if (!refreshDnsCache(now)) return false;

  WiFiClientSecure c;
  if (HTTPS_INSECURE) c.setInsecure();
  c.setTimeout(2000);

  bool ok = c.connect(cachedIP, HTTPS_PORT);
  if (!ok) {
    c.stop();
    return false;
  }

  c.print("HEAD / HTTP/1.1\r\nHost: ");
  c.print(HOST);
  c.print("\r\nConnection: close\r\n\r\n");

  uint32_t tRead0 = millis();
  while (c.connected() && millis() - tRead0 < 1200) {
    while (c.available()) (void)c.read();
    delay(5);
  }
  c.stop();
  return true;
}

// Open: send request, vent kort på status (valgfritt), logg timing uansett
int openDoorHttps_StatusCode(uint32_t now, bool &sentRequest, bool &gotStatus) {
  sentRequest = false;
  gotStatus = false;

  if (WiFi.status() != WL_CONNECTED) return 0;
  if (!refreshDnsCache(now)) return 0;

  WiFiClientSecure client;
  if (HTTPS_INSECURE) client.setInsecure();
  client.setTimeout(3500);

  if (!client.connect(cachedIP, HTTPS_PORT)) {
    client.stop();
    return 0;
  }

  // Path fra OPEN_URL
  const char* url = OPEN_URL;
  const char* path = strstr(url, "://");
  path = path ? strstr(path + 3, "/") : nullptr;
  if (!path) path = "/";

  client.print("GET ");
  client.print(path);
  client.print(" HTTP/1.1\r\nHost: ");
  client.print(HOST);
  client.print("\r\nConnection: close\r\nUser-Agent: esp32\r\nAccept: */*\r\n\r\n");
  sentRequest = true;

  // Forsøk å lese statuslinje i STATUS_WAIT_MS
  int httpCode = 0;
  uint32_t t0 = millis();
  String line;

  while (millis() - t0 < STATUS_WAIT_MS) {
    if (!client.connected() && !client.available()) break;

    while (client.available()) {
      line = client.readStringUntil('\n');
      line.trim();
      if (line.startsWith("HTTP/")) {
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);
        if (sp1 > 0) {
          String codeStr = (sp2 > sp1) ? line.substring(sp1 + 1, sp2) : line.substring(sp1 + 1);
          httpCode = codeStr.toInt();
          gotStatus = (httpCode > 0);
          client.stop();
          return httpCode;
        }
      }
    }
    delay(5);
  }

  // Ikke fått status, men request kan ha gått igjennom
  client.stop();
  return 0;
}

// =========================
// NTP + DEBUG WEBHOOK
// =========================
void ensureTimeNtp() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (ntpReady) return;

  configTime(TZ_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);

  for (int i = 0; i < 10; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      ntpReady = true;
      return;
    }
    delay(120);
  }
}

String isoTimeOrUptime() {
  if (!ntpReady) {
    uint32_t ms = millis();
    char b[32];
    snprintf(b, sizeof(b), "uptime=%lus", (unsigned long)(ms / 1000));
    return String(b);
  }

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  return String(buf);
}

bool postDebugToWebhook(const String& jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  if (DEBUG_WEBHOOK_INSECURE) client.setInsecure();
  client.setTimeout(4000);

  HTTPClient http;
  http.setTimeout(4000);

  if (!http.begin(client, DEBUG_WEBHOOK_URL)) return false;
  http.addHeader("Content-Type", "application/json");

  int code = http.POST((uint8_t*)jsonPayload.c_str(), jsonPayload.length());
  http.end();

  return (code > 0 && code < 400);
}

String buildDebugPayload(uint32_t now) {
  // Kortlinje
  String value;
  value.reserve(260);
  value += "dbg=ON";
  value += ",state="; value += stateName(st);
  value += ",cmRaw="; value += dbg.cmRawLast;
  value += ",cmFilt="; value += dbg.cmFiltLast;
  value += ",v="; value += String(dbg.vLast, 2);
  value += ",vFast="; value += String(dbg.vFastLast, 2);
  value += ",score="; value += dbg.scoreLast;
  value += ",fp="; value += dbg.fpCntLast;
  value += ",trig="; value += dbg.triggers;
  value += ",sent="; value += dbg.openSent;
  value += ",st="; value += dbg.openGotStatus;
  value += ",ok="; value += dbg.opensOk;
  value += ",fail="; value += dbg.opensFail;
  value += ",codeLast="; value += dbg.codeLast;
  value += ",openMsLast="; value += dbg.openMsLast;

  // Detaljer
  String msg;
  msg.reserve(800);
  msg += "time="; msg += isoTimeOrUptime(); msg += "\n";
  msg += "uptime_s="; msg += String(now / 1000); msg += "\n";

  msg += "wifi="; msg += (WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
  if (WiFi.status() == WL_CONNECTED) {
    msg += ",rssi="; msg += String(WiFi.RSSI());
    msg += ",ip="; msg += WiFi.localIP().toString();
  }
  msg += "\n";

  msg += "samples="; msg += dbg.samples;
  msg += ",presence="; msg += dbg.presenceCount;
  msg += ",hasRaw="; msg += dbg.hasRawCount;
  msg += "\n";

  msg += "cmRaw(min,max,last)=";
  msg += (dbg.cmRawMin == 99999 ? 0 : dbg.cmRawMin);
  msg += ","; msg += dbg.cmRawMax;
  msg += ","; msg += dbg.cmRawLast;
  msg += "\n";

  msg += "v(last,vFast,max)=";
  msg += String(dbg.vLast, 2);
  msg += ","; msg += String(dbg.vFastLast, 2);
  msg += ","; msg += String(dbg.vMax, 2);
  msg += "\n";

  msg += "open(sent,gotStatus,ok,fail,codeLast)=";
  msg += dbg.openSent; msg += ",";
  msg += dbg.openGotStatus; msg += ",";
  msg += dbg.opensOk; msg += ",";
  msg += dbg.opensFail; msg += ",";
  msg += dbg.codeLast;
  msg += "\n";

  msg += "openMs(min,avg,max,last)=";
  msg += (dbg.openMsMin == 999999 ? 0 : dbg.openMsMin);
  msg += ","; msg += dbg.openMsAvg();
  msg += ","; msg += dbg.openMsMax;
  msg += ","; msg += dbg.openMsLast;
  msg += "\n";

  msg += "warmups="; msg += dbg.warmups;
  msg += ",trackStart="; msg += TRACK_START_CM;
  msg += ",enter="; msg += ENTER_CM;
  msg += ",preEnter="; msg += PRE_ENTER_CM;
  msg += ",exit="; msg += EXIT_CM;
  msg += ",trackWindowMs="; msg += TRACK_WINDOW_MS;
  msg += ",cooldownMs="; msg += COOLDOWN_MS;
  msg += ",waitClearMs="; msg += WAIT_CLEAR_MS;
  msg += ",statusWaitMs="; msg += STATUS_WAIT_MS;

  // JSON escape
  String json;
  json.reserve(1300);
  json += "{\"value\":\"";
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '\\') json += "\\\\";
    else if (c == '"') json += "\\\"";
    else json += c;
  }
  json += "\",\"message\":\"";
  for (size_t i = 0; i < msg.length(); i++) {
    char c = msg[i];
    if (c == '\\') json += "\\\\";
    else if (c == '"') json += "\\\"";
    else if (c == '\n') json += "\\n";
    else json += c;
  }
  json += "\"}";
  return json;
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  ensureWifi();
  ensureTimeNtp();

  initRadar();

  dbg.reset(millis());
  setState(ST_ARMED, millis());
}

void loop() {
  ensureWifi();
  ensureTimeNtp();

  uint32_t now = millis();

  radar.read();

  if (now - lastRadarEvalMs < RADAR_EVAL_MS) {
    delay(2);
    return;
  }
  lastRadarEvalMs = now;

  RadarMeas rm = readRadar(now);

  bool hasRaw = (rm.presence && rm.chosenCm > 0);
  updateFilterAndVelocity(now, rm.chosenCm, hasRaw);

  int cmFilt = (hasRaw && isfinite(dFiltCm)) ? (int)lround(dFiltCm) : 0;
  int cmRaw  = (hasRaw) ? rm.chosenCm : 0;

  float v = lastVms;
  float vFast = lastVfast;

  // Warmup-gating
  if (!warmupArmed) {
    if (!rm.presence || cmFilt > WARMUP_OFF_CM) warmupArmed = true;
  } else {
    if (rm.presence && cmFilt > 0 && cmFilt <= WARMUP_ON_CM && (now - lastWarmupMs) > WARMUP_COOLDOWN_MS) {
      lastWarmupMs = now;
      warmupArmed = false;
      dbg.noteWarmup();
      (void)warmupHttpsHeadRoot(now);
    }
  }

  // sample stats
  dbg.noteSample(rm.presence, hasRaw, cmRaw, cmFilt, v, vFast, score, fastPassCount());

  // ---- State machine ----
  switch (st) {
    case ST_ARMED: {
      if (rm.presence && cmFilt > 0 && cmFilt <= TRACK_START_CM) {
        setState(ST_TRACKING, now);
      }
      break;
    }

    case ST_TRACKING: {
      if (now - trackSinceMs > TRACK_WINDOW_MS) {
        setState(ST_ARMED, now);
        break;
      }

      score = max(0, score - SCORE_DECAY);

      if (!(rm.presence && cmFilt > 0)) {
        pushFastPassEvidence(false);
        break;
      }

      // fast-pass: RAW avstand + vFast, 2 av siste 3
      bool fpOk = (cmRaw > 0 && cmRaw <= PRE_ENTER_CM && vFast > V_FAST_MS && vFast < V_MAX_MS);
      pushFastPassEvidence(fpOk);
      bool fastPass = (fastPassCount() >= 2);

      if (cmFilt <= ENTER_CM) score += 3;
      if (v > V_MIN_MS && v < V_MAX_MS) score += 3;
      if (rm.usedMoving) score += 1;
      if (!rm.usedMoving && cmFilt <= ENTER_CM && fabs(v) < 0.05f) score -= 2;

      if (score >= SCORE_TRIGGER || fastPass) {
        dbg.noteTrigger();
        cooldownSinceMs = now;

        dbg.noteOpenSent();
        uint32_t t0 = millis();
        bool sent = false, gotStatus = false;
        int code = openDoorHttps_StatusCode(now, sent, gotStatus);
        uint32_t ms = millis() - t0;
        dbg.noteOpenTiming(ms);
        if (gotStatus) dbg.noteOpenStatus(code);
        else dbg.codeLast = 0;

        setState(ST_COOLDOWN, now);
      }
      break;
    }

    case ST_COOLDOWN: {
      if (now - cooldownSinceMs >= COOLDOWN_MS) {
        setState(ST_WAIT_CLEAR, now);
      }
      break;
    }

    case ST_WAIT_CLEAR: {
      bool clearNow = (!rm.presence) || (rm.presence && cmFilt > EXIT_CM);

      if (clearNow) {
        if (clearSinceMs == 0) clearSinceMs = now;
        if (now - clearSinceMs >= WAIT_CLEAR_MS) {
          score = 0;
          lastRawCm = 0;
          dFiltCm = NAN;
          dFastCm = NAN;
          lastVms = 0.0f;
          lastVfast = 0.0f;
          fpRing[0] = fpRing[1] = fpRing[2] = false;
          fpIdx = 0;
          setState(ST_ARMED, now);
        }
      } else {
        clearSinceMs = 0;
      }
      break;
    }
  }

  // ---- Debug webhook push hver 30 sek ----
  if (WiFi.status() == WL_CONNECTED && (now - lastDebugPushMs) >= DEBUG_PUSH_MS) {
    lastDebugPushMs = now;

    String payload = buildDebugPayload(now);
    bool pushed = postDebugToWebhook(payload);

    if (pushed) {
      dbg.reset(now);
    }
  }

  delay(2);
}
