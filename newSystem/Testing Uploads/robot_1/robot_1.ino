/*
 * robot_1.ino
 * Basic UDP receiver firmware for Robot ID 1
 *
 * Receives UDP broadcast packets from the Python server.
 * Drives forward on RUN command, stops on STOP command.
 * Ignores position data for now — just proves UDP → motor pipeline works.
 *
 * Board  : ESP32-S3-DevKitC-1 v1.1
 * Motors : N20 6V via TB6612FNG driver
 * PWM cap: 70% (178/255) to protect motors from 7.2V battery
 *
 * Uses new LEDC API (ESP32 Arduino core >= 3.x)
 * ledcAttach(pin, freq, res) replaces ledcSetup + ledcAttachPin
 * ledcWrite(pin, duty)       replaces ledcWrite(channel, duty)
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ── Network Config ────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YourNetworkSSID";
const char* WIFI_PASSWORD = "YourNetworkPassword";

const uint16_t UDP_PORT    = 5005;
const uint8_t  MY_ROBOT_ID = 1;
// ─────────────────────────────────────────────────────────────────────────────

// ── Motor Driver Pins (TB6612FNG) ─────────────────────────────────────────────
#define PIN_PWMA   4
#define PIN_AIN1   6
#define PIN_AIN2   5
#define PIN_STBY   7
#define PIN_BIN1   8
#define PIN_BIN2   9
#define PIN_PWMB   10

// PWM config
#define PWM_FREQ   20000   // 20 kHz
#define PWM_RES    8       // 8-bit (0-255)
#define PWM_MAX    178     // 70% of 255 — protects motors from 7.2V battery
// ─────────────────────────────────────────────────────────────────────────────

// ── Packet format (must match udp.py exactly) ─────────────────────────────────
#define HEADER_SIZE      6    // command(1) + packet_number(4) + num_robots(1)
#define ROBOT_ENTRY_SIZE 13   // robot_id(1) + x(4) + y(4) + theta(4)
#define MAX_ROBOTS       10
#define MAX_PACKET_SIZE  (HEADER_SIZE + ROBOT_ENTRY_SIZE * MAX_ROBOTS)
// ─────────────────────────────────────────────────────────────────────────────

WiFiUDP  udp;
uint8_t  packetBuf[MAX_PACKET_SIZE];
uint32_t lastPacketNumber = UINT32_MAX;


// ── Big-endian helpers ────────────────────────────────────────────────────────
uint32_t readU32BE(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3]);
}

float readF32BE(const uint8_t* p) {
    uint32_t raw = readU32BE(p);
    float f;
    memcpy(&f, &raw, 4);
    return f;
}
// ─────────────────────────────────────────────────────────────────────────────


// ── Motor control ─────────────────────────────────────────────────────────────
void motorsStop() {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
    Serial.println("[Motor] STOP");
}

void motorsForward(uint8_t speed) {
    // Left motor forward:  AIN1 HIGH, AIN2 LOW
    // Right motor forward: BIN1 HIGH, BIN2 LOW
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PIN_PWMA, speed);
    ledcWrite(PIN_PWMB, speed);
    Serial.printf("[Motor] FORWARD speed=%d\n", speed);
}
// ─────────────────────────────────────────────────────────────────────────────


// ── Packet handler ────────────────────────────────────────────────────────────
void handlePacket(const uint8_t* buf, int len) {
    if (len < HEADER_SIZE) {
        Serial.println("[WARN] Packet too short.");
        return;
    }

    char     command      = (char)buf[0];
    uint32_t packetNumber = readU32BE(buf + 1);
    uint8_t  numRobots    = buf[5];

    // Discard stale packets (handles out-of-order UDP)
    if (lastPacketNumber != UINT32_MAX) {
        uint32_t gap = packetNumber - lastPacketNumber;
        if (gap > 0x80000000) {
            Serial.printf("[UDP] Stale packet #%lu discarded.\n", (unsigned long)packetNumber);
            return;
        }
    }
    lastPacketNumber = packetNumber;

    // STOP command — halt immediately
    if (command == 'S') {
        Serial.printf("[UDP] Pkt #%lu — STOP command received.\n", (unsigned long)packetNumber);
        motorsStop();
        return;
    }

    // RUN command — scan robot entries for our ID
    bool foundSelf = false;
    int offset = HEADER_SIZE;
    for (int i = 0; i < numRobots; i++) {
        if (offset + ROBOT_ENTRY_SIZE > len) break;

        uint8_t robotId = buf[offset];
        float   x       = readF32BE(buf + offset + 1);
        float   y       = readF32BE(buf + offset + 5);
        float   theta   = readF32BE(buf + offset + 9);

        if (robotId == MY_ROBOT_ID) {
            foundSelf = true;
            Serial.printf("[UDP] Pkt #%lu — RUN | x=%.3f y=%.3f theta=%.3f\n",
                (unsigned long)packetNumber, x, y, theta);
            motorsForward(PWM_MAX);
        }

        offset += ROBOT_ENTRY_SIZE;
    }

    if (!foundSelf) {
        // Our marker isn't visible to the camera — stop and wait
        Serial.printf("[UDP] Pkt #%lu — RUN but Robot %d not seen, stopping.\n",
            (unsigned long)packetNumber, MY_ROBOT_ID);
        motorsStop();
    }
}
// ─────────────────────────────────────────────────────────────────────────────


void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Boot] Robot 1 starting...");

    // Motor pins
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);

    // PWM — new API: ledcAttach(pin, freq, resolution)
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

    // Start with motors stopped and driver disabled
    motorsStop();
    digitalWrite(PIN_STBY, LOW);

    // WiFi
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

    // Enable motor driver now that we're connected
    digitalWrite(PIN_STBY, HIGH);
    Serial.println("[Motor] Driver enabled.");

    // UDP
    udp.begin(UDP_PORT);
    Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
    Serial.println("[Boot] Ready.");
}


void loop() {
    // If WiFi drops, stop motors and reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Lost connection — stopping motors.");
        motorsStop();
        digitalWrite(PIN_STBY, LOW);
        WiFi.reconnect();
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - t0 > 15000) ESP.restart();
            delay(500);
        }
        digitalWrite(PIN_STBY, HIGH);
        Serial.println("[WiFi] Reconnected.");
    }

    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        if (packetSize > MAX_PACKET_SIZE) packetSize = MAX_PACKET_SIZE;
        int len = udp.read(packetBuf, packetSize);
        if (len > 0) handlePacket(packetBuf, len);
    }
}
