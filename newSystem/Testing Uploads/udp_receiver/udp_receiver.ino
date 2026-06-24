/*
 * udp_receiver.ino
 * ESP32-S3 UDP Packet Receiver — Debug Tool
 *
 * Receives broadcast packets from the Python UDP server,
 * decodes them, prints to Serial, and mirrors output to a
 * webpage hosted at a static IP (192.168.0.201).
 *
 * Board : ESP32-S3-DevKitC-1 v1.1
 * Flash  : 8MB   PSRAM : 8MB   (N8R8 variant)
 *
 * Web monitor: http://192.168.0.201
 * Serial baud: 115200
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <math.h>

// ── Network Config ────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "RobotWifi";      // match udp.py
const char* WIFI_PASSWORD = "12345678";  // match udp.py

// Static IP for this ESP — must be on the same subnet as the server
IPAddress STATIC_IP(192, 168, 0, 201);
IPAddress GATEWAY  (192, 168, 0, 1);
IPAddress SUBNET   (255, 255, 255, 0);
IPAddress DNS      (8, 8, 8, 8);

const uint16_t UDP_PORT = 5005;   // match udp.py SERVER_PORT
// ─────────────────────────────────────────────────────────────────────────────

// ── Packet format (must match udp.py exactly) ─────────────────────────────────
// Header  : command (1 byte 'R'/'S') | packet_number (4 bytes BE) | num_robots (1 byte)
// Per robot: robot_id (1 byte) | x float (4 bytes BE) | y float (4 bytes BE) | theta float (4 bytes BE)
#define HEADER_SIZE      6    // 1 + 4 + 1
#define ROBOT_ENTRY_SIZE 13   // 1 + 4 + 4 + 4
#define MAX_ROBOTS       10
#define MAX_PACKET_SIZE  (HEADER_SIZE + ROBOT_ENTRY_SIZE * MAX_ROBOTS)  // 136 bytes
// ─────────────────────────────────────────────────────────────────────────────

// ── Log buffer (ring buffer for the web page) ─────────────────────────────────
#define LOG_LINES    30       // number of lines shown on the web page
#define LOG_LINE_LEN 120

char     logBuffer[LOG_LINES][LOG_LINE_LEN];
uint8_t  logHead  = 0;        // index of oldest line
uint8_t  logCount = 0;        // how many lines are filled
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void logLine(const char* line) {
    portENTER_CRITICAL(&logMux);
    strncpy(logBuffer[logHead], line, LOG_LINE_LEN - 1);
    logBuffer[logHead][LOG_LINE_LEN - 1] = '\0';
    logHead = (logHead + 1) % LOG_LINES;
    if (logCount < LOG_LINES) logCount++;
    portEXIT_CRITICAL(&logMux);

    Serial.println(line);
}
// ─────────────────────────────────────────────────────────────────────────────

WiFiUDP    udp;
WebServer  server(80);

uint8_t    packetBuf[MAX_PACKET_SIZE];
uint32_t   lastPacketNumber  = UINT32_MAX;   // sentinel: "not seen yet"
uint32_t   packetsReceived   = 0;
uint32_t   packetsDropped    = 0;


// ── Big-endian helpers ────────────────────────────────────────────────────────
uint32_t readU32BE(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3]);
}

float readF32BE(const uint8_t* p) {
    uint32_t raw = readU32BE(p);
    float    f;
    memcpy(&f, &raw, 4);
    return f;
}
// ─────────────────────────────────────────────────────────────────────────────


// ── Packet decoder ────────────────────────────────────────────────────────────
void decodeAndLog(const uint8_t* buf, int len) {
    char line[LOG_LINE_LEN];

    if (len < HEADER_SIZE) {
        logLine("[WARN] Packet too short for header.");
        return;
    }

    // Header
    char     command      = (char)buf[0];   // 'R' or 'S'
    uint32_t packetNumber = readU32BE(buf + 1);
    uint8_t  numRobots    = buf[5];

    // Packet-drop detection
    packetsReceived++;
    if (lastPacketNumber != UINT32_MAX) {
        uint32_t expected = (lastPacketNumber + 1);   // wraps naturally at 2^32
        if (packetNumber != expected) {
            uint32_t gap = packetNumber - lastPacketNumber;
            packetsDropped += (gap - 1);
            snprintf(line, sizeof(line),
                "[WARN] Gap! Expected #%lu got #%lu — ~%lu dropped. Total dropped: %lu",
                (unsigned long)expected,
                (unsigned long)packetNumber,
                (unsigned long)(gap - 1),
                (unsigned long)packetsDropped);
            logLine(line);
        }
    }
    lastPacketNumber = packetNumber;

    // Summary line
    snprintf(line, sizeof(line),
        "── Pkt #%lu | Cmd: %s | Robots: %d | Rcvd: %lu | Dropped: %lu",
        (unsigned long)packetNumber,
        (command == 'R') ? "RUN" : "STOP",
        numRobots,
        (unsigned long)packetsReceived,
        (unsigned long)packetsDropped);
    logLine(line);

    // Validate expected payload length
    int expectedLen = HEADER_SIZE + numRobots * ROBOT_ENTRY_SIZE;
    if (len < expectedLen) {
        snprintf(line, sizeof(line),
            "[WARN] Packet truncated: got %d bytes, expected %d for %d robots.",
            len, expectedLen, numRobots);
        logLine(line);
    }
    if (len > expectedLen) {
        snprintf(line, sizeof(line),
            "[WARN] %d unexpected leftover bytes — possible format mismatch.",
            len - expectedLen);
        logLine(line);
    }

    // Robot entries
    int offset = HEADER_SIZE;
    for (int i = 0; i < numRobots; i++) {
        if (offset + ROBOT_ENTRY_SIZE > len) break;

        uint8_t robotId = buf[offset];
        float   x       = readF32BE(buf + offset + 1);
        float   y       = readF32BE(buf + offset + 5);
        float   theta   = readF32BE(buf + offset + 9);
        float   thetaDeg = theta * (180.0f / M_PI);

        snprintf(line, sizeof(line),
            "  Robot %2d  x=%+.3fm  y=%+.3fm  θ=%+.1f°  (%.4frad)",
            robotId, x, y, thetaDeg, theta);
        logLine(line);

        offset += ROBOT_ENTRY_SIZE;
    }
}
// ─────────────────────────────────────────────────────────────────────────────


// ── Web page ──────────────────────────────────────────────────────────────────
void handleRoot() {
    // Build the log section
    String logHtml = "";
    portENTER_CRITICAL(&logMux);
    uint8_t start = (logCount < LOG_LINES) ? 0 : logHead;
    for (uint8_t i = 0; i < logCount; i++) {
        uint8_t idx = (start + i) % LOG_LINES;
        logHtml += logBuffer[idx];
        logHtml += "\n";
    }
    portEXIT_CRITICAL(&logMux);

    String page = R"rawhtml(<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta http-equiv="refresh" content="1">
  <title>ESP32 UDP Monitor</title>
  <style>
    body  { background:#111; color:#0f0; font-family:monospace; font-size:13px; margin:16px; }
    h2    { color:#4af; margin-bottom:4px; }
    .info { color:#aaa; font-size:11px; margin-bottom:12px; }
    pre   { background:#1a1a1a; border:1px solid #333; padding:12px;
            height:520px; overflow-y:auto; white-space:pre-wrap; word-break:break-all; }
  </style>
</head>
<body>
  <h2>ESP32 UDP Packet Monitor</h2>
  <div class="info">
    IP: )rawhtml";
    page += WiFi.localIP().toString();
    page += R"rawhtml( &nbsp;|&nbsp; Port: )rawhtml";
    page += String(UDP_PORT);
    page += R"rawhtml( &nbsp;|&nbsp; Received: )rawhtml";
    page += String(packetsReceived);
    page += R"rawhtml( &nbsp;|&nbsp; Dropped: )rawhtml";
    page += String(packetsDropped);
    page += R"rawhtml( &nbsp;|&nbsp; <span style="color:#fa0">Auto-refreshes every 1s</span>
  </div>
  <pre>)rawhtml";
    page += logHtml;
    page += R"rawhtml(</pre>
</body>
</html>)rawhtml";

    server.send(200, "text/html", page);
}
// ─────────────────────────────────────────────────────────────────────────────


void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Boot] ESP32-S3 UDP Receiver starting...");

    // Static IP
    if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS)) {
        Serial.println("[WiFi] Static IP config failed.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > 15000) {
            Serial.println("\n[WiFi] Timeout — restarting.");
            ESP.restart();
        }
        delay(500);
        Serial.print(".");
    }

    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Expected static IP: %s\n", STATIC_IP.toString().c_str());

    // UDP
    udp.begin(UDP_PORT);
    Serial.printf("[UDP]  Listening on port %d\n", UDP_PORT);

    // Web server
    server.on("/", handleRoot);
    server.begin();
    Serial.printf("[Web]  Server started at http://%s\n", WiFi.localIP().toString().c_str());

    logLine("[Boot] Ready — waiting for UDP packets...");
}


void loop() {
    // Handle web requests
    server.handleClient();

    // Check for UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        if (packetSize > MAX_PACKET_SIZE) {
            char warn[LOG_LINE_LEN];
            snprintf(warn, sizeof(warn),
                "[WARN] Packet too large (%d bytes) — truncating to %d.",
                packetSize, MAX_PACKET_SIZE);
            logLine(warn);
            packetSize = MAX_PACKET_SIZE;
        }

        int len = udp.read(packetBuf, packetSize);
        if (len > 0) {
            decodeAndLog(packetBuf, len);
        }
    }
}
