#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ld2410.h>
#include <time.h>

#include "secrets.h"   // Secrets live here, Wi-Fi SSID, backend URLs, TLS policy flags.
                       // This file is gitignored, see secrets.example.h in the repo.

// ============================================================
// 24 GHz radar door opener, ESP32-S2 (LOLIN S2 Mini)
//
// Design intent, in one sentence:
//   Detect an approaching person reliably, then trigger an HTTPS "open door" call
//   early enough that the door finishes opening right before the person arrives.
//
// Key real-world constraint:
//   The dominant delay is NOT the radar, it's network + backend latency (~1.6–1.7 s),
//   so we deliberately trigger multiple meters before the handle.
//
// Notes for humans and other AIs reading this:
//   - This is a small real-time system with a state machine.
//   - The radar is sampled continuously, we "evaluate" at ~40 Hz for stable timing.
//   - The open request is treated as effectively fire-and-forget, we only wait a
//     short time for the HTTP status line, because the physical door actuation can
//     happen after the ESP has already moved on.
// ============================================================

// =========================
// SETTINGS
// =========================
// These compile-time flags gate Serial prints.
// Keep them off in production, printing can create jitter and timing artifacts.
#define DEBUG_NET   0
#define DEBUG_RADAR 0
#define DEBUG_TIME  0

// Optional DNS servers, helps some DHCP networks behave consistently.
IPAddress DNS1(1,1,1,1);
IPAddress DNS2(8,8,8,8);

// Backend port
static const uint16_t HTTPS_PORT = 443;

// Radar pins (as physically soldered)
// NOTE, Arduino Serial1 "RX pin" is the MCU pin that receives data, so it connects
//       to the radar TX. Same for TX pin, it connects to radar RX.
#define RADAR_RX_PIN 33   // MCU RX, connected to radar TX
#define RADAR_TX_PIN 18   // MCU TX, connected to radar RX
#define RADAR_BAUD   256000

// Timing
static const uint32_t WIFI_RETRY_MS   = 3000;
static const uint32_t RADAR_EVAL_MS   = 25;      // 40 Hz evaluation loop (read() can be faster)
static const uint32_t COOLDOWN_MS     = 5000;    // after triggering, ignore everything for a bit
static const uint32_t WAIT_CLEAR_MS   = 1800;    // require continuous "clear" before re-arming
static const uint32_t TRACK_WINDOW_MS = 1400;    // max time we allow a track to reach a decision

// Trigger tuning (cm)
// The numbers below are not "sensor range", they are tuned for door opening timing
// given measured network/backend latency, and the physical approach path.
static const int ENTER_CM       = 340;
static const int PRE_ENTER_CM   = 550;  // tuned for ~1.6 s open latency, used by fast-pass
static const int EXIT_CM        = 470;
static const int TRACK_START_CM = 650;

// Sensor hygiene
// Radar frames can occasionally jump due to multipath, people turning, reflections, etc.
// We treat big single-step jumps as invalid to avoid spurious triggers.
static const int MIN_VALID_CM = 55;
static const int MAX_VALID_CM = 650;
static const int JUMP_CM      = 160;

// Approach / speed thresholds
// We estimate speed from distance deltas, positive v means approaching (distance decreasing).
static const float V_MIN_MS  = 0.25f;
static const float V_FAST_MS = 0.38f; // fast-pass evidence, 2 of last 3 samples
static const float V_MAX_MS  = 3.00f;

// Score-based triggering
// This is a soft decision mechanism, multiple weak signals accumulate into a trigger.
static const int SCORE_TRIGGER = 8;
static const int SCORE_DECAY   = 1;

// Warmup
// Establishing TLS + TCP can be expensive, and first request after idle sometimes
// suffers extra delay (DNS, ARP, Wi-Fi power save, TLS session setup, etc).
// Warmup sends a cheap HTTPS HEAD to "/" when we see "someone is far away", so that
// by the time they reach the door, the network path is already hot.
static const int WARMUP_ON_CM            = 950;
static const int WARMUP_OFF_CM           = 1200;
static const uint32_t WARMUP_COOLDOWN_MS = 6000;

// Debug webhook push
// Headless, periodic observability, intended for Home Assistant or similar receiver.
static const uint32_t DEBUG_PUSH_MS = 30000;

// NTP
// We use NTP mainly for human-readable timestamps in debug logs,
// not for control decisions.
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.nist.gov";
static const long  TZ_OFFSET_SEC = 3600;
static const int   DST_OFFSET_SEC = 0;

// For sane stats (avoid absurd spikes from a single glitch sample)
static const float V_STATS_CAP = 6.0f;

// How long we try to read the HTTP status line after sending the open request.
// Real-world behavior, the door still opens even if we never receive a status line,
// so we treat "no status" as "unknown", not as "failed".
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

// State machine for controlling trigger logic and preventing re-triggers.
// - ARMED, ready to start tracking when someone enters the far range
// - TRACKING, we evaluate approach evidence, and may trigger
// - COOLDOWN, we triggered, ignore everything for a fixed time
// - WAIT_CLEAR, require continuous absence or retreat before re-arming
enum State : uint8_t { ST_ARMED, ST_TRACKING, ST_COOLDOWN, ST_WAIT_CLEAR };
State st = ST_ARMED;

// Core timing markers
uint32_t lastWifiAttemptMs = 0;
uint32_t lastRadarEvalMs   = 0;

uint32_t trackSinceMs      = 0;
uint32_t cooldownSinceMs   = 0;
uint32_t clearSinceMs      = 0;

// Score accumulator used in ST_TRACKING
int score = 0;

// Distance filter
// lastRawCm is used for jump rejection.
// dFiltCm is a low-pass filtered distance used for stable "position" decisions.
int   lastRawCm   = 0;
float dFiltCm     = NAN;

// Velocity tracking (normal, derived from filtered distance)
// lastVms is the higher stability estimate.
float lastVms = 0.0f;

// fast-pass path, lower-latency filter and velocity
// This is intentionally "twitchier" than the normal filter,
// it is used for quick detection of fast approaches.
float dFastCm = NAN;
float lastVfast = 0.0f;

// fast-pass evidence, ring buffer of last 3 "fast-pass ok" booleans.
// We trigger fast-pass if at least 2 of the last 3 samples are ok,
// this rejects one-off glitches without adding too much latency.
bool fpRing[3] = {false, false, false};
uint8_t fpIdx = 0;

// warmup gating state
bool warmupArmed = true;
uint32_t lastWarmupMs = 0;

// cached DNS resolution, to avoid repeated DNS lookups inside time-critical paths
IPAddress cachedIP;
bool hasCachedIP = false;
uint32_t lastDnsMs = 0;

// NTP readiness flag
bool ntpReady = false;

// Debug push timer
uint32_t lastDebugPushMs = 0;

// =========================
// DEBUG STATS SNAPSHOT
// =========================
// This struct is a rolling "telemetry window".
// It is periodically posted to a webhook, then reset on successful send.
// We intentionally avoid storing large buffers, this is meant to run headless.
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

  // Open tracking, request send vs status reception.
  // openSent counts requests we attempted to send.
  // openGotStatus counts how often we managed to parse an HTTP status line.
  uint32_t openSent = 0;
  uint32_t openGotStatus = 0;
  uint32_t opensOk = 0;
  uint32_t opensFail = 0;
  int      codeLast = 0;

  // Client-side timing of the open call.
  // NOTE, this is not the physical door-open time, only our local measurement.
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

    // Track max absolute velocity, with a cap to avoid single-sample nonsense.
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
    else if (code != 0) opensFail++; // code==0 means "no status received"
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

  // Entering TRACKING resets short-term decision evidence.
  // We want each track attempt to start fresh, no leftover score or fast-pass residue.
  if (ns == ST_TRACKING) {
    trackSinceMs = millis();
    score = 0;
    fpRing[0] = fpRing[1] = fpRing[2] = false;
    fpIdx = 0;
  }

  // WAIT_CLEAR uses clearSinceMs as a "continuous clear" timer.
  // Reset to 0 so that the first clear sample starts the timer cleanly.
  if (ns == ST_WAIT_CLEAR) {
    clearSinceMs = 0;
  }
}

void ensureWifi() {
  // Keep Wi-Fi connection alive, but do not block the loop too often.
  // The radar and state machine can keep running even if Wi-Fi is down,
  // but we obviously cannot open the door without connectivity.
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = now;

  WiFi.mode(WIFI_STA);

  // Avoid DNS surprises, some networks provide flaky DNS.
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, DNS1, DNS2);
  WiFi.begin(WIFI_SSID);

  // Short blocking connect window, then return to loop.
  // This is a trade-off, we want reconnection attempts, but we do not want
  // multi-second stalls repeatedly.
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(100);
}

bool initRadar() {
  // Radar is read via UART.
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(Serial1);

  // Startup sanity check, try to observe frames for up to 2 seconds.
  // If we never see frames, wiring or baud is likely wrong.
  uint32_t t0 = millis();
  bool sawFrame = false;
  while (millis() - t0 < 2000) {
    if (radar.read()) sawFrame = true;
    delay(5);
  }
  return sawFrame;
}

struct RadarMeas {
  bool presence;    // overall presence flag from the module
  bool mDet;        // moving target detected
  bool sDet;        // stationary target detected
  int  mCm;         // moving distance (cm)
  int  sCm;         // stationary distance (cm)
  int  chosenCm;    // chosen best distance (cm), 0 if none or rejected
  bool usedMoving;  // whether chosen distance came from moving track
};

static inline bool saneDist(int cm) {
  return (cm >= MIN_VALID_CM && cm <= MAX_VALID_CM);
}

int chooseDistancePreferred(bool mDet, int mCm, bool sDet, int sCm, bool &usedMoving) {
  // Prefer moving distance when available, because we care most about approach.
  // If only stationary is available (person stopped), use that as a fallback.
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

  // If the module says "no presence", we do not trust distances.
  if (!out.presence) return out;

  int raw = chooseDistancePreferred(out.mDet, out.mCm, out.sDet, out.sCm, out.usedMoving);

  // Jump rejection:
  // If distance suddenly jumps a lot compared to previous raw sample,
  // reject this sample, it is usually reflection noise or a transient.
  if (raw > 0 && lastRawCm > 0 && abs(raw - lastRawCm) > JUMP_CM) {
    raw = 0;
  }
  if (raw > 0) lastRawCm = raw;

  out.chosenCm = raw;
  return out;
}

void updateFilterAndVelocity(uint32_t now, int rawCm, bool hasRaw) {
  // We run two filters:
  // - dFiltCm, "stable" low-pass for score logic, less jitter
  // - dFastCm, "fast" low-pass for quick changes, used by fast-pass
  //
  // Both filters are simple IIR (exponential smoothing), good enough and cheap.
  // Velocity is then computed from filter deltas, not from raw samples.

  static float prevFilt = NAN;
  static uint32_t prevMs = 0;

  if (hasRaw) {
    if (!isfinite(dFiltCm)) dFiltCm = (float)rawCm;
    else {
      const float a = 0.55f; // lower a means smoother but more latency
      dFiltCm = a * (float)rawCm + (1.0f - a) * dFiltCm;
    }

    if (!isfinite(dFastCm)) dFastCm = (float)rawCm;
    else {
      const float af = 0.80f; // higher af means faster response, more noise
      dFastCm = af * (float)rawCm + (1.0f - af) * dFastCm;
    }
  }

  // Velocity from stable filter
  // v > 0 means distance is decreasing, person approaching.
  if (hasRaw && isfinite(dFiltCm) && isfinite(prevFilt) && prevMs != 0) {
    float dt = (now - prevMs) / 1000.0f;
    // dt bounds, protect against weird timing (Wi-Fi stalls, clock jumps, etc)
    if (dt > 0.015f && dt < 0.6f) {
      lastVms = ((prevFilt - dFiltCm) / 100.0f) / dt;
    } else {
      lastVms = 0.0f;
    }
  } else {
    lastVms = 0.0f;
  }

  // Velocity from fast filter, lower latency, used for "fast approach" evidence
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
  // Count true values in the 3-sample ring buffer.
  // 2 of 3 is a simple majority vote, robust against a single glitch.
  int c = 0;
  for (int i = 0; i < 3; i++) if (fpRing[i]) c++;
  return c;
}

void pushFastPassEvidence(bool ok) {
  fpRing[fpIdx] = ok;
  fpIdx = (fpIdx + 1) % 3;
}

bool refreshDnsCache(uint32_t now) {
  // DNS resolution can be slow or flaky, and we do not want to do it inside
  // the most time-critical part unless we have to.
  // We cache the resolved IP for a short time, and re-resolve periodically.
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
  // Purpose:
  //   Reduce perceived door latency by "waking" network path and TLS stack
  //   before we actually need to open the door.
  //
  // Behavior:
  //   Do a minimal HTTPS connect and HEAD request to "/" and then close.
  //   Even if the response is ignored, the connection attempt primes DNS, ARP,
  //   Wi-Fi, and sometimes the TLS session cache.
  if (!refreshDnsCache(now)) return false;

  WiFiClientSecure c;
  if (HTTPS_INSECURE) c.setInsecure(); // controlled environment, trust is external
  c.setTimeout(2000);

  bool ok = c.connect(cachedIP, HTTPS_PORT);
  if (!ok) {
    c.stop();
    return false;
  }

  c.print("HEAD / HTTP/1.1\r\nHost: ");
  c.print(HOST);
  c.print("\r\nConnection: close\r\n\r\n");

  // Drain any available response bytes briefly, then close.
  uint32_t tRead0 = millis();
  while (c.connected() && millis() - tRead0 < 1200) {
    while (c.available()) (void)c.read();
    delay(5);
  }
  c.stop();
  return true;
}

// Open: send request, optionally read status quickly, always return fast
int openDoorHttps_StatusCode(uint32_t now, bool &sentRequest, bool &gotStatus) {
  sentRequest = false;
  gotStatus = false;

  // Without Wi-Fi, we cannot do anything meaningful.
  if (WiFi.status() != WL_CONNECTED) return 0;
  if (!refreshDnsCache(now)) return 0;

  WiFiClientSecure client;
  if (HTTPS_INSECURE) client.setInsecure();
  client.setTimeout(3500);

  // Connect to cached IP, not hostname, avoids DNS in the hot path.
  if (!client.connect(cachedIP, HTTPS_PORT)) {
    client.stop();
    return 0;
  }

  // Extract path from OPEN_URL.
  // OPEN_URL can be "https://host/path?query...", we only need the "/path..."
  const char* url = OPEN_URL;
  const char* path = strstr(url, "://");
  path = path ? strstr(path + 3, "/") : nullptr;
  if (!path) path = "/";

  // Minimal HTTP GET request.
  // We intentionally keep headers small, less time and fewer bytes.
  client.print("GET ");
  client.print(path);
  client.print(" HTTP/1.1\r\nHost: ");
  client.print(HOST);
  client.print("\r\nConnection: close\r\nUser-Agent: esp32\r\nAccept: */*\r\n\r\n");
  sentRequest = true;

  // Attempt to parse HTTP status line quickly.
  // If we do not see it, we return 0 (unknown), but the server may still have acted.
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

  // No status line within the window, request may still succeed.
  client.stop();
  return 0;
}

// =========================
// NTP + DEBUG WEBHOOK
// =========================
void ensureTimeNtp() {
  // We only attempt NTP when connected.
  // NTP is used for timestamps in debug payloads, not for control decisions,
  // so failure to get NTP is not critical.
  if (WiFi.status() != WL_CONNECTED) return;
  if (ntpReady) return;

  configTime(TZ_OFFSET_SEC, DST_OFFSET_SEC, NTP1, NTP2);

  // Wait briefly for a plausible epoch.
  for (int i = 0; i < 10; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) { // sanity threshold, roughly 2023+
      ntpReady = true;
      return;
    }
    delay(120);
  }
}

String isoTimeOrUptime() {
  // Use ISO time when NTP is ready, otherwise include uptime for ordering.
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
  // Headless telemetry, post a compact JSON to a webhook endpoint.
  // The webhook can store it, forward to Home Assistant, etc.
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
  // Payload shape:
  // - "value", a short one-line key=value string (easy to glance at)
  // - "message", a multi-line detail block (human readable)
  //
  // We keep this compact and explicit so you can debug without serial console.

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

  // Include the most important tuning constants for "remote forensics".
  msg += "warmups="; msg += dbg.warmups;
  msg += ",trackStart="; msg += TRACK_START_CM;
  msg += ",enter="; msg += ENTER_CM;
  msg += ",preEnter="; msg += PRE_ENTER_CM;
  msg += ",exit="; msg += EXIT_CM;
  msg += ",trackWindowMs="; msg += TRACK_WINDOW_MS;
  msg += ",cooldownMs="; msg += COOLDOWN_MS;
  msg += ",waitClearMs="; msg += WAIT_CLEAR_MS;
  msg += ",statusWaitMs="; msg += STATUS_WAIT_MS;

  // JSON escape "value" and "message".
  // This avoids pulling in a JSON library, keeps dependencies minimal.
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

  // Attempt Wi-Fi early so NTP and DNS warmup have a chance.
  ensureWifi();
  ensureTimeNtp();

  // Start radar UART and confirm that frames are flowing.
  initRadar();

  dbg.reset(millis());
  setState(ST_ARMED, millis());
}

void loop() {
  // Keep connectivity and time services up, without letting them dominate the loop.
  ensureWifi();
  ensureTimeNtp();

  uint32_t now = millis();

  // ld2410 library requires frequent read() calls to update internal state.
  // We call it every loop iteration, but we only "evaluate" at RADAR_EVAL_MS.
  radar.read();

  // Maintain a stable evaluation cadence (40 Hz).
  // This gives consistent velocity dt and predictable logic behavior.
  if (now - lastRadarEvalMs < RADAR_EVAL_MS) {
    delay(2);
    return;
  }
  lastRadarEvalMs = now;

  // Read and sanitize one radar measurement for this evaluation tick.
  RadarMeas rm = readRadar(now);

  // hasRaw means we have a plausible distance sample we accept this tick.
  bool hasRaw = (rm.presence && rm.chosenCm > 0);

  // Update filters and velocity estimates.
  // Even if hasRaw is false, velocity is forced to 0 for safety.
  updateFilterAndVelocity(now, rm.chosenCm, hasRaw);

  int cmFilt = (hasRaw && isfinite(dFiltCm)) ? (int)lround(dFiltCm) : 0;
  int cmRaw  = (hasRaw) ? rm.chosenCm : 0;

  float v = lastVms;
  float vFast = lastVfast;

  // Warmup-gating logic
  // Goal, perform one warmup request when we see someone in a "far" band.
  // - warmupArmed prevents repeated warmups in a short time.
  // - WARMUP_OFF_CM prevents re-arming while the person is still in range.
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

  // Telemetry sample, taken every evaluation tick.
  dbg.noteSample(rm.presence, hasRaw, cmRaw, cmFilt, v, vFast, score, fastPassCount());

  // ---- State machine ----
  switch (st) {
    case ST_ARMED: {
      // We enter TRACKING when presence appears within a distance band.
      // TRACK_START_CM is the "far threshold", this defines when we start caring.
      if (rm.presence && cmFilt > 0 && cmFilt <= TRACK_START_CM) {
        setState(ST_TRACKING, now);
      }
      break;
    }

    case ST_TRACKING: {
      // Hard timeout, do not track forever.
      // This prevents "stuck tracking" on noisy presence.
      if (now - trackSinceMs > TRACK_WINDOW_MS) {
        setState(ST_ARMED, now);
        break;
      }

      // Score decay means you must keep supplying evidence to trigger,
      // it prevents a single weak hint from accumulating forever.
      score = max(0, score - SCORE_DECAY);

      // If we lose presence or distance, we cannot build reliable evidence this tick.
      if (!(rm.presence && cmFilt > 0)) {
        pushFastPassEvidence(false);
        break;
      }

      // fast-pass: raw distance + fast velocity evidence
      // This is a "low latency lane" that can trigger even if score is still building.
      // Criteria:
      //   - raw distance within PRE_ENTER_CM
      //   - fast velocity above V_FAST_MS
      //   - 2 of last 3 samples meet criteria (majority vote)
      bool fpOk = (cmRaw > 0 && cmRaw <= PRE_ENTER_CM && vFast > V_FAST_MS && vFast < V_MAX_MS);
      pushFastPassEvidence(fpOk);
      bool fastPass = (fastPassCount() >= 2);

      // Score features:
      // - close enough, stronger evidence (ENTER_CM)
      // - approaching at plausible speed, stronger evidence
      // - moving target chosen, slight bonus, moving is usually "intent"
      // - stationary chosen while close and almost no movement, slight penalty,
      //   this helps avoid triggers from a person standing near but not approaching
      if (cmFilt <= ENTER_CM) score += 3;
      if (v > V_MIN_MS && v < V_MAX_MS) score += 3;
      if (rm.usedMoving) score += 1;
      if (!rm.usedMoving && cmFilt <= ENTER_CM && fabs(v) < 0.05f) score -= 2;

      // Decision point:
      // If either the score passes threshold OR fastPass says "high confidence fast approach",
      // we trigger one open request and move to COOLDOWN.
      if (score >= SCORE_TRIGGER || fastPass) {
        dbg.noteTrigger();
        cooldownSinceMs = now;

        // "Open door" is intentionally not a blocking transaction.
        // We send the request, then wait briefly for status (optional).
        // The physical door system can respond after we have moved on.
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
      // Simple fixed cooldown.
      // Avoids repeated opens when a person lingers in the beam.
      if (now - cooldownSinceMs >= COOLDOWN_MS) {
        setState(ST_WAIT_CLEAR, now);
      }
      break;
    }

    case ST_WAIT_CLEAR: {
      // Re-arm only when the scene is "clear" for WAIT_CLEAR_MS continuously.
      // clearNow is true if:
      //   - no presence, or
      //   - presence exists but is far away (greater than EXIT_CM)
      //
      // EXIT_CM is the hysteresis threshold, prevents quick re-arm bounce.
      bool clearNow = (!rm.presence) || (rm.presence && cmFilt > EXIT_CM);

      if (clearNow) {
        if (clearSinceMs == 0) clearSinceMs = now;
        if (now - clearSinceMs >= WAIT_CLEAR_MS) {
          // Reset tracking state, gives a clean slate for the next approach.
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
        // Someone is still "in the scene", restart the clear timer.
        clearSinceMs = 0;
      }
      break;
    }
  }

  // ---- Debug webhook push every 30 seconds ----
  // Telemetry is pushed opportunistically.
  // On success, we reset the snapshot, so the next report covers a new window.
  if (WiFi.status() == WL_CONNECTED && (now - lastDebugPushMs) >= DEBUG_PUSH_MS) {
    lastDebugPushMs = now;

    String payload = buildDebugPayload(now);
    bool pushed = postDebugToWebhook(payload);

    if (pushed) {
      dbg.reset(now);
    }
  }

  // Small delay, reduces CPU burn without harming timing much.
  delay(2);
}
