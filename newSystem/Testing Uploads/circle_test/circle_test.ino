/*
 * circle_test.ino
 * Robot ID #1 — Circle Test
 *
 * Receives (x, y, theta) from the UDP broadcast server and uses
 * closed-loop control to drive the robot in a circle of TARGET_RADIUS_M
 * around the world origin (0, 0).
 *
 * Strategy (two phases):
 *   ALIGN  – Turn on the spot to face a point on the circle, then drive there.
 *   ORBIT  – Once on the circle (within RADIUS_DEADBAND), track the CCW tangent.
 *
 * Hardware: ESP32-S3-DEVKITC-1, TB6612FNG motor driver, N20 encoders
 *
 * Key parameters to tune:
 *   TARGET_RADIUS_M   – orbit radius (metres)
 *   BASE_SPEED        – forward PWM during orbit
 *   KP_HEADING        – heading correction gain (orbit phase)
 *   KP_RADIUS         – radius correction gain  (orbit phase)
 *   ALIGN_KP          – heading gain while driving to the circle
 *   APPROACH_SPEED    – forward PWM while approaching the circle
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <math.h>

// ── Wi-Fi / Network ──────────────────────────────────────────────────────────
const char* SSID     = "RobotWifi";
const char* PASSWORD = "12345678";
const int   UDP_PORT = 5005;

// ── Robot Identity ────────────────────────────────────────────────────────────
const uint8_t MY_ROBOT_ID = 1;

// ── Motor Driver Pins (TB6612FNG) ─────────────────────────────────────────────
#define PWMA_PIN   4
#define AIN1_PIN   6
#define AIN2_PIN   5
#define PWMB_PIN   10
#define BIN1_PIN   8
#define BIN2_PIN   9
#define STBY_PIN   7

// ── PWM Config ────────────────────────────────────────────────────────────────
// Motors rated 6V, battery 7.2V — cap at 70% duty = 178/255
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_MAX         178

// ── Control Parameters ────────────────────────────────────────────────────────
const float TARGET_RADIUS_M  = 0.10f;   // Orbit radius in metres (10 cm)
const float RADIUS_DEADBAND  = 0.025f;  // Enter orbit when within ±2.5 cm of target radius

// Orbit phase
const int   BASE_SPEED       = 100;     // Forward PWM during orbit (tune down if too fast)
const float KP_HEADING       = 60.0f;  // Heading error → differential
const float KP_RADIUS        = 50.0f;  // Radius error  → differential

// Approach phase (driving to the circle)
const int   APPROACH_SPEED   = 90;     // Forward PWM while approaching
const float ALIGN_KP         = 70.0f;  // Heading gain during approach
const float ALIGN_DEADBAND   = 0.15f;  // rad — don't steer if heading error is tiny

// ── Packet Format (matches udp.py) ───────────────────────────────────────────
#define HEADER_SIZE      6
#define ROBOT_ENTRY_SIZE 13

// ── Globals ───────────────────────────────────────────────────────────────────
WiFiUDP  udp;
uint8_t  packetBuf[256];

float    g_x     = 0.0f;
float    g_y     = 0.0f;
float    g_theta = 0.0f;
bool     g_positionValid    = false;
bool     g_run              = false;
uint32_t g_lastPacketNumber = 0xFFFFFFFF;

// Phase tracking
enum Phase { SPIN_SEARCH, APPROACH, ORBIT };
Phase g_phase = SPIN_SEARCH;

// ── Helpers ───────────────────────────────────────────────────────────────────
uint32_t readUInt32BE(const uint8_t* buf) {
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
         | ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
}

float readFloatBE(const uint8_t* buf) {
    uint32_t raw = readUInt32BE(buf);
    float val;
    memcpy(&val, &raw, sizeof(float));
    return val;
}

float wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// ── Motor Control ─────────────────────────────────────────────────────────────
void setMotorLeft(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);
    if (speed >= 0) { digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW); }
    else            { digitalWrite(AIN1_PIN, LOW);  digitalWrite(AIN2_PIN, HIGH); speed = -speed; }
    ledcWrite(PWMA_PIN, speed);
}

void setMotorRight(int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);
    if (speed >= 0) { digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, LOW); }
    else            { digitalWrite(BIN1_PIN, LOW);  digitalWrite(BIN2_PIN, HIGH); speed = -speed; }
    ledcWrite(PWMB_PIN, speed);
}

void stopMotors() {
    digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, HIGH); ledcWrite(PWMA_PIN, 0);
    digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, HIGH); ledcWrite(PWMB_PIN, 0);
}

// ── Parse UDP Packet ──────────────────────────────────────────────────────────
void parsePacket(const uint8_t* buf, int len) {
    if (len < HEADER_SIZE) return;

    uint8_t  command      = buf[0];
    uint32_t packetNumber = readUInt32BE(buf + 1);
    uint8_t  numRobots    = buf[5];

    if (packetNumber == g_lastPacketNumber) return;
    g_lastPacketNumber = packetNumber;

    g_run = (command == 'R');

    int offset = HEADER_SIZE;
    for (int i = 0; i < numRobots; i++) {
        if (offset + ROBOT_ENTRY_SIZE > len) break;
        uint8_t robot_id = buf[offset];
        float   x        = readFloatBE(buf + offset + 1);
        float   y        = readFloatBE(buf + offset + 5);
        float   theta    = readFloatBE(buf + offset + 9);
        offset += ROBOT_ENTRY_SIZE;

        if (robot_id == MY_ROBOT_ID) {
            g_x = x; g_y = y; g_theta = theta;
            g_positionValid = true;
        }
    }
}

// ── Phase: Spin slowly until camera sees the marker ──────────────────────────
void doSpinSearch() {
    setMotorLeft(35);
    setMotorRight(-35);
}

// ── Phase: Drive toward the nearest point on the target circle ───────────────
//
// Target point: the point on the circle (radius R) closest to the robot,
// i.e. in the direction of the robot from the origin.
// Desired heading = angle from robot toward that point.
void doApproach() {
    float radius = sqrtf(g_x * g_x + g_y * g_y);

    // Pick the point on the circle in the direction of the robot from origin
    float angle_to_robot = atan2f(g_y, g_x);
    float target_x = TARGET_RADIUS_M * cosf(angle_to_robot);
    float target_y = TARGET_RADIUS_M * sinf(angle_to_robot);

    // Desired heading: from robot position toward that target point
    float dx = target_x - g_x;
    float dy = target_y - g_y;
    float desired_theta = atan2f(dy, dx);
    float heading_error = wrapAngle(desired_theta - g_theta);

    // If heading error is large, turn on the spot first
    if (fabsf(heading_error) > 0.5f) {
        // Pure rotation to align
        int turn = (int)(ALIGN_KP * heading_error);
        turn = constrain(turn, -PWM_MAX, PWM_MAX);
        setMotorLeft(-turn);
        setMotorRight(turn);
        return;
    }

    // Aligned — drive forward with correction
    float diff = ALIGN_KP * heading_error;
    diff = constrain(diff, -(float)APPROACH_SPEED, (float)APPROACH_SPEED);

    int left_speed  = constrain((int)(APPROACH_SPEED - diff), 0, PWM_MAX);
    int right_speed = constrain((int)(APPROACH_SPEED + diff), 0, PWM_MAX);

    setMotorLeft(left_speed);
    setMotorRight(right_speed);

    Serial.printf("[Approach] r=%.3f | target=(%.3f,%.3f) | hErr=%.2f | L=%d R=%d\n",
        radius, target_x, target_y, heading_error, left_speed, right_speed);
}

// ── Phase: Orbit the origin CCW ──────────────────────────────────────────────
void doOrbit() {
    float radius = sqrtf(g_x * g_x + g_y * g_y);

    // CCW tangent direction at the robot's current position
    float radial_angle  = atan2f(g_y, g_x);
    float desired_theta = wrapAngle(radial_angle + (float)M_PI / 2.0f);
    float heading_error = wrapAngle(desired_theta - g_theta);
    float radius_error  = radius - TARGET_RADIUS_M;

    // Positive diff → turns robot left (CCW); negative → turns right (CW)
    // Heading error corrects direction; radius error nudges inward/outward
    float diff = (KP_HEADING * heading_error) - (KP_RADIUS * radius_error);
    diff = constrain(diff, -(float)PWM_MAX, (float)PWM_MAX);

    int left_speed  = constrain((int)(BASE_SPEED - diff / 2.0f), 0, PWM_MAX);
    int right_speed = constrain((int)(BASE_SPEED + diff / 2.0f), 0, PWM_MAX);

    setMotorLeft(left_speed);
    setMotorRight(right_speed);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 400) {
        lastPrint = millis();
        Serial.printf(
            "[Orbit] x=%.3f y=%.3f r=%.3f | des=%.2f hErr=%.2f rErr=%.3f | L=%d R=%d\n",
            g_x, g_y, radius, desired_theta, heading_error, radius_error, left_speed, right_speed
        );
    }
}

// ── Main Control Dispatcher ───────────────────────────────────────────────────
void circleControl() {
    if (!g_positionValid) {
        g_phase = SPIN_SEARCH;
        doSpinSearch();
        return;
    }

    float radius = sqrtf(g_x * g_x + g_y * g_y);
    float radius_error = fabsf(radius - TARGET_RADIUS_M);

    // State transitions
    switch (g_phase) {
        case SPIN_SEARCH:
            // Got a fix — decide whether to approach or go straight to orbit
            if (radius_error < RADIUS_DEADBAND) {
                g_phase = ORBIT;
                Serial.println("[Phase] → ORBIT (already on circle)");
            } else {
                g_phase = APPROACH;
                Serial.println("[Phase] → APPROACH");
            }
            break;

        case APPROACH:
            if (radius_error < RADIUS_DEADBAND) {
                g_phase = ORBIT;
                Serial.println("[Phase] → ORBIT");
            }
            break;

        case ORBIT:
            // If we've drifted badly off the circle, re-approach
            if (radius_error > RADIUS_DEADBAND * 3.0f) {
                g_phase = APPROACH;
                Serial.println("[Phase] → APPROACH (radius drift)");
            }
            break;
    }

    // Execute current phase
    switch (g_phase) {
        case SPIN_SEARCH: doSpinSearch(); break;
        case APPROACH:    doApproach();   break;
        case ORBIT:       doOrbit();      break;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Robot] Booting — Circle Test");

    pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, LOW);

    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

    Serial.printf("[Robot] Connecting to %s ...\n", SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print('.'); }
    Serial.printf("\n[Robot] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    udp.begin(UDP_PORT);
    Serial.printf("[Robot] Listening on UDP port %d\n", UDP_PORT);

    digitalWrite(STBY_PIN, HIGH);
    Serial.println("[Robot] Ready. Waiting for RUN command...");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0 && packetSize <= (int)sizeof(packetBuf)) {
        udp.read(packetBuf, packetSize);
        parsePacket(packetBuf, packetSize);
    }

    if (g_run) {
        circleControl();
    } else {
        stopMotors();
        g_positionValid = false;
        g_phase = SPIN_SEARCH;
    }

    delay(20);
}
