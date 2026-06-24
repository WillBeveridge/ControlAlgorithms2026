/*
 * go_to_origin.ino
 * Robot ID #1
 *
 * Reads (x, y, theta) from the UDP broadcast server and drives
 * to (0,0), stops, then re-navigates if disturbed.
 *
 * Phases:
 *   SEARCH  — no position fix yet, spin slowly
 *   ALIGN   — pulse-turn to face origin
 *   DRIVE   — smooth forward drive with continuous heading correction
 *   ARRIVED — stopped at origin, watching for disturbance
 *
 * Hardware: ESP32-S3-DEVKITC-1, TB6612FNG motor driver
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Arduino.h>
#include <math.h>

// ── Wi-Fi / Network ──────────────────────────────────────────────────────────
const char* SSID     = "RobotWifi";
const char* PASSWORD = "12345678";
const int   UDP_PORT = 5005;

// ── Robot Identity ────────────────────────────────────────────────────────────
const uint8_t MY_ROBOT_ID = 1;

// ── Motor Driver Pins ─────────────────────────────────────────────────────────
#define PWMA_PIN   4
#define AIN1_PIN   6
#define AIN2_PIN   5
#define PWMB_PIN   10
#define BIN1_PIN   8
#define BIN2_PIN   9
#define STBY_PIN   7

// ── PWM Config ────────────────────────────────────────────────────────────────
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_MAX         178

// ── Tuning ────────────────────────────────────────────────────────────────────
const float STOP_RADIUS      = 0.03f;  // Arrived when within 3 cm
const float RESET_RADIUS     = 0.06f;  // Re-navigate if moved more than 6 cm away
const float ALIGN_THRESH     = 0.35f;  // rad — switch from align to drive
const float REALIGN_THRESH   = 0.6f;   // rad — re-align mid-drive if heading drifts this far

const int   TURN_SPEED       = 120;    // PWM during turn pulse
const int   TURN_PULSE_MS    = 150;    // ms per turn pulse
const int   TURN_WAIT_MS     = 400;    // ms to wait after pulse (camera update)

const int   MAX_DRIVE_SPEED  = 130;    // Max forward PWM
const int   MIN_DRIVE_SPEED  = 80;     // Min forward PWM (keeps moving at close range)
const float KP_STEER         = 80.0f; // Heading correction gain while driving
const float THETA_OFFSET     = 0.0f;  // Adjust if robot faces wrong way (try M_PI)

// ── Packet Format ─────────────────────────────────────────────────────────────
#define HEADER_SIZE      6
#define ROBOT_ENTRY_SIZE 13

// ── Log Buffer ────────────────────────────────────────────────────────────────
#define LOG_LINES    80
#define LOG_LINE_LEN 96
static char  g_log[LOG_LINES][LOG_LINE_LEN];
static int   g_logHead  = 0;
static int   g_logCount = 0;

void logLine(const char* msg) {
    Serial.println(msg);
    strncpy(g_log[g_logHead], msg, LOG_LINE_LEN - 1);
    g_log[g_logHead][LOG_LINE_LEN - 1] = '\0';
    g_logHead = (g_logHead + 1) % LOG_LINES;
    if (g_logCount < LOG_LINES) g_logCount++;
}
void logf(const char* fmt, ...) {
    char buf[LOG_LINE_LEN];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    logLine(buf);
}

// ── Web Server ────────────────────────────────────────────────────────────────
WebServer server(80);

String buildLog() {
    String s = "";
    if (g_logCount < LOG_LINES)
        for (int i = 0; i < g_logCount; i++) s += String(g_log[i]) + "\n";
    else
        for (int i = 0; i < LOG_LINES; i++) s += String(g_log[(g_logHead+i)%LOG_LINES]) + "\n";
    return s;
}
void handleRoot() {
    String html = R"rawhtml(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Robot Monitor</title>
<style>
  body{background:#111;color:#0f0;font-family:monospace;font-size:13px;margin:0;padding:10px;}
  h2{color:#0f0;margin:0 0 6px;}
  #log{white-space:pre-wrap;word-break:break-all;height:calc(100vh - 60px);
       overflow-y:auto;border:1px solid #333;padding:8px;box-sizing:border-box;}
</style></head><body>
<h2>Robot #)rawhtml";
    html += String(MY_ROBOT_ID);
    html += R"rawhtml( — Live Log</h2>
<div id="log">)rawhtml";
    html += buildLog();
    html += R"rawhtml(</div>
<script>
  var atBottom=true,d=document.getElementById('log');
  d.scrollTop=d.scrollHeight;
  d.addEventListener('scroll',function(){atBottom=d.scrollTop+d.clientHeight>=d.scrollHeight-5;});
  setInterval(function(){fetch('/log').then(r=>r.text()).then(t=>{d.textContent=t;if(atBottom)d.scrollTop=d.scrollHeight;});},500);
</script></body></html>)rawhtml";
    server.send(200, "text/html", html);
}
void handleLog() { server.send(200, "text/plain", buildLog()); }

// ── Globals ───────────────────────────────────────────────────────────────────
WiFiUDP  udp;
uint8_t  packetBuf[256];

float    g_x             = 0.0f;
float    g_y             = 0.0f;
float    g_theta         = 0.0f;
bool     g_positionValid = false;
bool     g_run           = false;
uint32_t g_lastPacket    = 0xFFFFFFFF;

enum Phase { SEARCH, ALIGN, DRIVE, ARRIVED };
Phase g_phase = SEARCH;

static unsigned long g_lastLog   = 0;
static unsigned long g_turnTimer = 0;
static bool          g_inWait    = false;  // true = waiting after a turn pulse

// ── Helpers ───────────────────────────────────────────────────────────────────
uint32_t readUInt32BE(const uint8_t* b) {
    return ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|(uint32_t)b[3];
}
float readFloatBE(const uint8_t* b) {
    uint32_t r = readUInt32BE(b); float v; memcpy(&v,&r,4); return v;
}
float wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f*M_PI;
    while (a < -M_PI) a += 2.0f*M_PI;
    return a;
}

// ── Motor Control ─────────────────────────────────────────────────────────────
void setMotorLeft(int s) {
    s = constrain(s, -PWM_MAX, PWM_MAX);
    if (s >= 0) { digitalWrite(AIN1_PIN,HIGH); digitalWrite(AIN2_PIN,LOW); }
    else        { digitalWrite(AIN1_PIN,LOW);  digitalWrite(AIN2_PIN,HIGH); s=-s; }
    ledcWrite(PWMA_PIN, s);
}
void setMotorRight(int s) {
    s = constrain(s, -PWM_MAX, PWM_MAX);
    if (s >= 0) { digitalWrite(BIN1_PIN,HIGH); digitalWrite(BIN2_PIN,LOW); }
    else        { digitalWrite(BIN1_PIN,LOW);  digitalWrite(BIN2_PIN,HIGH); s=-s; }
    ledcWrite(PWMB_PIN, s);
}
void stopMotors() {
    digitalWrite(AIN1_PIN,HIGH); digitalWrite(AIN2_PIN,HIGH); ledcWrite(PWMA_PIN,0);
    digitalWrite(BIN1_PIN,HIGH); digitalWrite(BIN2_PIN,HIGH); ledcWrite(PWMB_PIN,0);
}

// ── Parse UDP Packet ──────────────────────────────────────────────────────────
void parsePacket(const uint8_t* buf, int len) {
    if (len < HEADER_SIZE) return;
    uint8_t  cmd  = buf[0];
    uint32_t pnum = readUInt32BE(buf+1);
    uint8_t  n    = buf[5];
    if (pnum == g_lastPacket) return;
    g_lastPacket = pnum;
    g_run = (cmd == 'R');
    int offset = HEADER_SIZE;
    for (int i = 0; i < n; i++) {
        if (offset + ROBOT_ENTRY_SIZE > len) break;
        uint8_t id = buf[offset];
        float x    = readFloatBE(buf+offset+1);
        float y    = readFloatBE(buf+offset+5);
        float th   = readFloatBE(buf+offset+9);
        offset += ROBOT_ENTRY_SIZE;
        if (id == MY_ROBOT_ID) {
            g_x = x; g_y = y;
            g_theta = wrapAngle(th + THETA_OFFSET);
            g_positionValid = true;
        }
    }
}

// ── Navigation ────────────────────────────────────────────────────────────────
void navigate() {
    unsigned long now = millis();

    // ── SEARCH ───────────────────────────────────────────────────────────────
    if (!g_positionValid) {
        g_phase = SEARCH;
        setMotorLeft(50); setMotorRight(-50);
        if (now - g_lastLog > 500) { g_lastLog=now; logLine("[Nav] Searching..."); }
        return;
    }

    float dist          = sqrtf(g_x*g_x + g_y*g_y);
    float desired_theta = atan2f(-g_y, -g_x);
    float heading_error = wrapAngle(desired_theta - g_theta);

    // ── ARRIVED — check for disturbance ──────────────────────────────────────
    if (g_phase == ARRIVED) {
        if (dist > RESET_RADIUS) {
            logf("[Nav] Disturbed (dist=%.3f) — re-navigating", dist);
            g_phase   = ALIGN;
            g_inWait  = false;
        } else {
            stopMotors();
            if (now - g_lastLog > 2000) { g_lastLog=now; logf("[Nav] Holding origin (dist=%.3f)", dist); }
        }
        return;
    }

    // ── Check arrival ─────────────────────────────────────────────────────────
    if (dist < STOP_RADIUS) {
        stopMotors();
        if (g_phase != ARRIVED) {
            g_phase = ARRIVED;
            logf("[Nav] Arrived! dist=%.3f", dist);
        }
        return;
    }

    // ── ALIGN — pulse-turn to face origin ────────────────────────────────────
    if (g_phase == SEARCH || g_phase == ALIGN) {
        g_phase = ALIGN;

        if (fabsf(heading_error) <= ALIGN_THRESH) {
            g_phase  = DRIVE;
            g_inWait = false;
            logf("[Nav] Aligned — driving (hErr=%.2f)", heading_error);
            return;
        }

        if (!g_inWait) {
            int turn = (heading_error > 0) ? TURN_SPEED : -TURN_SPEED;
            setMotorLeft(-turn); setMotorRight(turn);
            if (now - g_turnTimer > TURN_PULSE_MS) {
                stopMotors();
                g_turnTimer = now;
                g_inWait    = true;
            }
        } else {
            stopMotors();
            if (now - g_turnTimer > TURN_WAIT_MS) {
                g_turnTimer = now;
                g_inWait    = false;
            }
        }

        if (now - g_lastLog > 300) {
            g_lastLog = now;
            logf("[Nav] ALIGN  dist=%.3f theta=%.2f desired=%.2f hErr=%.2f",
                 dist, g_theta, desired_theta, heading_error);
        }
        return;
    }

    // ── DRIVE — smooth forward with continuous heading correction ─────────────
    if (g_phase == DRIVE) {

        // Re-align if heading drifts too far
        if (fabsf(heading_error) > REALIGN_THRESH) {
            g_phase  = ALIGN;
            g_inWait = false;
            logf("[Nav] Re-aligning (hErr=%.2f)", heading_error);
            return;
        }

        // Scale speed down as we get close so we don't overshoot
        float speed_scale = constrain(dist / 0.15f, 0.0f, 1.0f);
        int base_speed = (int)(MIN_DRIVE_SPEED + speed_scale * (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED));

        float correction = KP_STEER * heading_error;
        correction = constrain(correction, -(float)base_speed, (float)base_speed);

        int left  = constrain((int)(base_speed - correction), 0, PWM_MAX);
        int right = constrain((int)(base_speed + correction), 0, PWM_MAX);

        setMotorLeft(left); setMotorRight(right);

        if (now - g_lastLog > 300) {
            g_lastLog = now;
            logf("[Nav] DRIVE  dist=%.3f hErr=%.2f spd=%d L=%d R=%d",
                 dist, heading_error, base_speed, left, right);
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    logLine("[Robot] Booting — Go To Origin");

    pinMode(AIN1_PIN,OUTPUT); pinMode(AIN2_PIN,OUTPUT);
    pinMode(BIN1_PIN,OUTPUT); pinMode(BIN2_PIN,OUTPUT);
    pinMode(STBY_PIN,OUTPUT);
    digitalWrite(STBY_PIN, LOW);

    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

    IPAddress localIP(192,168,0,200);
    IPAddress gateway(192,168,0,1);
    IPAddress subnet(255,255,255,0);
    IPAddress dns(8,8,8,8);
    WiFi.config(localIP, gateway, subnet, dns);

    logf("[Robot] Connecting to %s ...", SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print('.'); }
    Serial.println();

    logf("[Robot] Connected! IP: %s", WiFi.localIP().toString().c_str());
    logf("[Robot] Open http://%s/ in your browser", WiFi.localIP().toString().c_str());

    server.on("/",    handleRoot);
    server.on("/log", handleLog);
    server.begin();
    logLine("[Web]   HTTP server started");

    udp.begin(UDP_PORT);
    logf("[Robot] Listening on UDP port %d", UDP_PORT);

    digitalWrite(STBY_PIN, HIGH);
    logLine("[Robot] Ready. Waiting for RUN command...");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    int sz = udp.parsePacket();
    if (sz > 0 && sz <= (int)sizeof(packetBuf)) {
        udp.read(packetBuf, sz);
        parsePacket(packetBuf, sz);
    }

    if (g_run) {
        navigate();
    } else {
        stopMotors();
        g_positionValid = false;
        g_phase = SEARCH;
    }

    delay(20);
}
