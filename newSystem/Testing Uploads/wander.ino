/*
 * wander.ino — Random Wander with Two-Level Collision Avoidance
 * ESP32-S3-DEVKITC-1 v1.1
 *
 * Overview
 * ────────
 * The robot wanders in a random direction until either an arena boundary or
 * another robot enters one of two concentric safety zones:
 *
 *   WARN zone  → robot slows to WARN_SPEED and begins steering away
 *   HARD zone  → robot stops, then reverses-steers at REVERSE_SPEED
 *                (this boundary is never crossed)
 *
 * Both zones apply to:
 *   • Arena edges  (world coords: X ∈ [-ARENA_W, +ARENA_W],
 *                                  Y ∈ [-ARENA_H, +ARENA_H])
 *   • Every other robot (distance between centres)
 *
 * Position data arrives via UDP broadcast from the Python server (udp.py).
 * Packet format (big-endian):
 *   [0]      command      : 1 byte  ('R' = run, 'S' = stop)
 *   [1..4]   packet_number: uint32
 *   [5]      num_robots   : uint8
 *   for each robot:
 *     [+0]   robot_id     : uint8
 *     [+1..4] x           : float32  (metres, right = positive)
 *     [+5..8] y           : float32  (metres, up    = positive)
 *     [+9..12] theta      : float32  (radians, CCW  = positive)
 *
 * Hardware (matches System_specifications.pdf pin assignments exactly)
 * ────────────────────────────────────────────────────────────────────
 *   Left  motor  : AIN1=GPIO6, AIN2=GPIO5, PWMA=GPIO4
 *   Right motor  : BIN1=GPIO8, BIN2=GPIO9, PWMB=GPIO10
 *   STBY         : GPIO7
 *   Left  encoder: C1=GPIO1 (ch A), C2=GPIO2  (ch B) → PCNT_UNIT_0
 *   Right encoder: C1=GPIO21(ch A), C2=GPIO47 (ch B) → PCNT_UNIT_1
 *   Battery ADC  : GPIO18
 *
 * Motor wiring note: Left motor M1→MOTORA2, M2→MOTORA1 so AIN1 HIGH,
 * AIN2 LOW = forward. Right motor M1→MOTORB1, M2→MOTORB2 so BIN1 HIGH,
 * BIN2 LOW = forward. Both match the spec.
 *
 * PWM is limited to MAX_PWM (≤178 / 255 ≈ 70%) to protect the 6V-rated
 * motors from the 7.2V battery supply.
 */

// ── Includes ────────────────────────────────────────────────────────────────
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/pcnt.h"
#include <math.h>

// ── Network — must match udp.py constants ───────────────────────────────────
static const char*    WIFI_SSID      = "YourNetworkSSID";
static const char*    WIFI_PASSWORD  = "YourNetworkPassword";
static const uint16_t UDP_PORT       = 5005;

// ── Robot identity ───────────────────────────────────────────────────────────
// Set this to match the ArUco marker ID physically attached to this robot.
static const uint8_t MY_ROBOT_ID = 0;

// ── Arena bounds (metres from world origin) ──────────────────────────────────
// Must match SCENE_WIDTH_M / SCENE_HEIGHT_M in tracker.py:
//   SCENE_WIDTH_M = 2.0  → half-width  = 1.000 m
//   SCENE_HEIGHT_M = 1.125 → half-height = 0.5625 m
static constexpr float ARENA_W = 1.000f;   // ±X limit (metres)
static constexpr float ARENA_H = 0.5625f;  // ±Y limit (metres)

// ── Collision / avoidance zones ──────────────────────────────────────────────
// Robot–robot distances (centre to centre, metres)
static constexpr float ROBOT_WARN_DIST = 0.25f;  // outer zone: slow + steer
static constexpr float ROBOT_HARD_DIST = 0.12f;  // inner zone: stop + reverse

// Distance from this robot's centre to the nearest arena edge (metres)
static constexpr float EDGE_WARN_DIST  = 0.20f;  // outer zone
static constexpr float EDGE_HARD_DIST  = 0.08f;  // inner zone

// ── Motor GPIO pins ──────────────────────────────────────────────────────────
static constexpr int PIN_AIN1 = 6;
static constexpr int PIN_AIN2 = 5;
static constexpr int PIN_PWMA = 4;
static constexpr int PIN_BIN1 = 8;
static constexpr int PIN_BIN2 = 9;
static constexpr int PIN_PWMB = 10;
static constexpr int PIN_STBY = 7;

// ── Encoder GPIO pins ────────────────────────────────────────────────────────
static constexpr int PIN_ENC_LA = 1;   // Left  encoder channel A
static constexpr int PIN_ENC_LB = 2;   // Left  encoder channel B
static constexpr int PIN_ENC_RA = 21;  // Right encoder channel A
static constexpr int PIN_ENC_RB = 47;  // Right encoder channel B

// ── PWM configuration ────────────────────────────────────────────────────────
static constexpr int   PWM_FREQ       = 20000;   // 20 kHz (matches spec)
static constexpr int   PWM_RESOLUTION = 8;       // 8-bit (0–255)
static constexpr int   PWM_CHAN_LEFT  = 0;       // LEDC channel for left  motor
static constexpr int   PWM_CHAN_RIGHT = 1;       // LEDC channel for right motor
static constexpr uint8_t MAX_PWM      = 178;     // 70% of 255 ≈ 178
static constexpr uint8_t FULL_SPEED   = 160;     // normal wander speed  (≤ MAX_PWM)
static constexpr uint8_t WARN_SPEED   = 90;      // speed inside WARN zone
static constexpr uint8_t REVERSE_SPEED = 120;    // speed when reversing in HARD zone

// ── Wander timing ────────────────────────────────────────────────────────────
static constexpr uint32_t WANDER_CHANGE_MS   = 3000;  // max ms before new random heading
static constexpr uint32_t TURN_DURATION_MS   = 600;   // how long a deliberate turn lasts
static constexpr uint32_t REVERSE_DURATION_MS = 400;  // how long reverse lasts in HARD zone

// ── UDP packet layout ────────────────────────────────────────────────────────
// Header: 1 (cmd) + 4 (pkt_num) + 1 (num_robots) = 6 bytes
static constexpr int PKT_HEADER_SIZE  = 6;
// Per-robot entry: 1 (id) + 4 (x) + 4 (y) + 4 (theta) = 13 bytes
static constexpr int PKT_ROBOT_SIZE   = 13;
static constexpr int MAX_ROBOTS       = 20;
static constexpr int UDP_BUF_SIZE     = PKT_HEADER_SIZE + MAX_ROBOTS * PKT_ROBOT_SIZE + 4;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

struct RobotPose {
    float x;
    float y;
    float theta;
    bool  valid;   // true once we have received at least one update
};

// ── State machine ─────────────────────────────────────────────────────────────
enum class DriveState {
    WANDER,   // Normal: driving straight in current heading
    WARN,     // WARN zone: slowing and steering away
    HARD,     // HARD zone: stopping then reversing-steering
    STOP,     // Server sent STOP command
};

// ─────────────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────────────

static WiFiUDP          udp;
static uint8_t          udpBuf[UDP_BUF_SIZE];

// Own pose (updated from UDP)
static RobotPose        myPose    = {0.0f, 0.0f, 0.0f, false};
// All other robots seen in the last packet
static RobotPose        others[MAX_ROBOTS];
static uint8_t          otherIds[MAX_ROBOTS];
static uint8_t          numOthers = 0;

// Server command
static volatile bool    serverRun = false;  // true when 'R' received

// Avoidance state
static DriveState       driveState        = DriveState::STOP;
static float            wanderHeading     = 0.0f;  // radians, world frame
static uint32_t         lastHeadingChange = 0;
static uint32_t         stateEnteredAt    = 0;

// Avoidance direction: +1.0 = turn left, -1.0 = turn right
static float            avoidDir          = 1.0f;

// Latest received packet number (to discard stale packets)
static uint32_t         lastPacketNum = 0;
static bool             firstPacket   = true;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: read a big-endian float from a byte buffer
// ─────────────────────────────────────────────────────────────────────────────
static float readBEFloat(const uint8_t* buf) {
    uint32_t raw = ((uint32_t)buf[0] << 24)
                 | ((uint32_t)buf[1] << 16)
                 | ((uint32_t)buf[2] << 8)
                 |  (uint32_t)buf[3];
    float val;
    memcpy(&val, &raw, sizeof(val));
    return val;
}

static uint32_t readBEUint32(const uint8_t* buf) {
    return ((uint32_t)buf[0] << 24)
         | ((uint32_t)buf[1] << 16)
         | ((uint32_t)buf[2] << 8)
         |  (uint32_t)buf[3];
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor control
// ─────────────────────────────────────────────────────────────────────────────

/*
 * setMotor — drive one motor at a given duty (0–MAX_PWM).
 * dir: 1 = forward, -1 = backward, 0 = coast.
 * pinA, pinB: direction pins. pinPWM: LEDC channel index.
 *
 * Per spec:
 *   Forward : IN1 HIGH, IN2 LOW
 *   Backward: IN1 LOW,  IN2 HIGH
 *   Coast   : IN1 LOW,  IN2 LOW
 *   Brake   : IN1 HIGH, IN2 HIGH  (not used here)
 */
static void setMotor(int pinA, int pinB, int pwmChan, int dir, uint8_t duty) {
    uint8_t clampedDuty = (duty > MAX_PWM) ? MAX_PWM : duty;
    if (dir > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
    } else if (dir < 0) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
    } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        clampedDuty = 0;
    }
    ledcWrite(pwmChan, clampedDuty);
}

/*
 * drive — set both motors with differential steering.
 * leftDuty / rightDuty: signed, negative = backward.
 */
static void drive(int leftDuty, int rightDuty) {
    int ld = constrain(leftDuty,  -MAX_PWM, MAX_PWM);
    int rd = constrain(rightDuty, -MAX_PWM, MAX_PWM);
    setMotor(PIN_AIN1, PIN_AIN2, PWM_CHAN_LEFT,  (ld > 0) ? 1 : (ld < 0) ? -1 : 0, (uint8_t)abs(ld));
    setMotor(PIN_BIN1, PIN_BIN2, PWM_CHAN_RIGHT, (rd > 0) ? 1 : (rd < 0) ? -1 : 0, (uint8_t)abs(rd));
}

static void stopMotors() {
    drive(0, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// PCNT (hardware pulse counter) setup
// ─────────────────────────────────────────────────────────────────────────────
static void initEncoders() {
    // Left motor encoder → PCNT_UNIT_0
    // Channel 0 = encoder A, Channel 1 = encoder B (logic flipped per spec)
    pcnt_config_t cfgL0 = {
        .pulse_gpio_num  = PIN_ENC_LA,
        .ctrl_gpio_num   = PIN_ENC_LB,
        .lctrl_mode      = PCNT_MODE_REVERSE,
        .hctrl_mode      = PCNT_MODE_KEEP,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   =  32767,
        .counter_l_lim   = -32767,
        .unit            = PCNT_UNIT_0,
        .channel         = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfgL0);

    pcnt_config_t cfgL1 = {
        .pulse_gpio_num  = PIN_ENC_LB,
        .ctrl_gpio_num   = PIN_ENC_LA,
        .lctrl_mode      = PCNT_MODE_KEEP,
        .hctrl_mode      = PCNT_MODE_REVERSE,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   =  32767,
        .counter_l_lim   = -32767,
        .unit            = PCNT_UNIT_0,
        .channel         = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&cfgL1);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // Right motor encoder → PCNT_UNIT_1
    pcnt_config_t cfgR0 = {
        .pulse_gpio_num  = PIN_ENC_RA,
        .ctrl_gpio_num   = PIN_ENC_RB,
        .lctrl_mode      = PCNT_MODE_REVERSE,
        .hctrl_mode      = PCNT_MODE_KEEP,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   =  32767,
        .counter_l_lim   = -32767,
        .unit            = PCNT_UNIT_1,
        .channel         = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfgR0);

    pcnt_config_t cfgR1 = {
        .pulse_gpio_num  = PIN_ENC_RB,
        .ctrl_gpio_num   = PIN_ENC_RA,
        .lctrl_mode      = PCNT_MODE_KEEP,
        .hctrl_mode      = PCNT_MODE_REVERSE,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   =  32767,
        .counter_l_lim   = -32767,
        .unit            = PCNT_UNIT_1,
        .channel         = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&cfgR1);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi + UDP setup
// ─────────────────────────────────────────────────────────────────────────────
static void initWiFi() {
    Serial.print("[WiFi] Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    Serial.print("[UDP ] Listening on port ");
    Serial.println(UDP_PORT);
}

// ─────────────────────────────────────────────────────────────────────────────
// UDP packet parsing
// ─────────────────────────────────────────────────────────────────────────────
static void parsePacket(const uint8_t* buf, int len) {
    // Minimum size: 6-byte header + at least 0 robot entries
    if (len < PKT_HEADER_SIZE) return;

    char     cmd       = (char)buf[0];
    uint32_t pktNum    = readBEUint32(buf + 1);
    uint8_t  numRobots = buf[5];

    // Discard out-of-order packets (handle uint32 wrap-around)
    if (!firstPacket) {
        int32_t diff = (int32_t)(pktNum - lastPacketNum);
        if (diff <= 0) return;   // older or duplicate packet
    }
    firstPacket   = false;
    lastPacketNum = pktNum;

    // Update server run/stop command
    serverRun = (cmd == 'R');

    // Parse robot entries
    numOthers = 0;
    for (int i = 0; i < numRobots; i++) {
        int offset = PKT_HEADER_SIZE + i * PKT_ROBOT_SIZE;
        if (offset + PKT_ROBOT_SIZE > len) break;

        uint8_t rid   = buf[offset];
        float   rx    = readBEFloat(buf + offset + 1);
        float   ry    = readBEFloat(buf + offset + 5);
        float   rth   = readBEFloat(buf + offset + 9);

        if (rid == MY_ROBOT_ID) {
            myPose = {rx, ry, rth, true};
        } else {
            if (numOthers < MAX_ROBOTS) {
                otherIds[numOthers] = rid;
                others[numOthers]   = {rx, ry, rth, true};
                numOthers++;
            }
        }
    }
}

static void pollUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0 && packetSize <= UDP_BUF_SIZE) {
        udp.read(udpBuf, packetSize);
        parsePacket(udpBuf, packetSize);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Threat assessment
// ─────────────────────────────────────────────────────────────────────────────

/*
 * edgeClearance — returns the distance (metres) from (x,y) to the nearest
 * arena wall. Positive means inside the arena.
 */
static float edgeClearance(float x, float y) {
    float dx = ARENA_W - fabsf(x);   // distance to nearest X wall
    float dy = ARENA_H - fabsf(y);   // distance to nearest Y wall
    return fminf(dx, dy);
}

/*
 * nearestEdgeEscapeAngle — returns the world-frame heading that points most
 * directly away from the nearest wall(s).
 */
static float nearestEdgeEscapeAngle(float x, float y) {
    float dx = ARENA_W - fabsf(x);
    float dy = ARENA_H - fabsf(y);
    // Push away from the closer axis
    if (dx < dy) {
        // Closer to left or right wall
        return (x < 0.0f) ? 0.0f : (float)M_PI;   // escape rightward or leftward
    } else {
        // Closer to top or bottom wall
        return (y < 0.0f) ? (float)(M_PI / 2.0)   // escape upward
                          : (float)(-M_PI / 2.0);  // escape downward
    }
}

/*
 * nearestRobotDist — distance to the closest other robot (metres).
 * Returns a large value if no others are known.
 */
static float nearestRobotDist(float x, float y, float* escapeAngleOut) {
    float minDist = 1e9f;
    float ex = 0.0f, ey = 0.0f;

    for (uint8_t i = 0; i < numOthers; i++) {
        if (!others[i].valid) continue;
        float ddx = x - others[i].x;
        float ddy = y - others[i].y;
        float d   = sqrtf(ddx * ddx + ddy * ddy);
        if (d < minDist) {
            minDist = d;
            // Escape heading = direction from other robot towards us
            ex = ddx;
            ey = ddy;
        }
    }

    if (escapeAngleOut) {
        *escapeAngleOut = (minDist < 1e8f) ? atan2f(ey, ex) : wanderHeading;
    }
    return minDist;
}

// ─────────────────────────────────────────────────────────────────────────────
// Differential-drive heading control
// ─────────────────────────────────────────────────────────────────────────────

/*
 * headingError — signed angle from current heading to target heading.
 * Result is in [-π, π].
 */
static float headingError(float currentTheta, float targetTheta) {
    float err = targetTheta - currentTheta;
    while (err >  (float)M_PI)  err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI)  err += 2.0f * (float)M_PI;
    return err;
}

/*
 * driveToHeading — produce left/right PWM values to steer towards a target
 * heading at the given base speed. A proportional turn correction is added.
 * Output is written directly to the motors.
 *
 * When we don't yet have a valid pose we fall back to open-loop turns using
 * avoidDir so the robot still reacts.
 */
static void driveToHeading(float targetTheta, uint8_t baseSpeed) {
    if (!myPose.valid) {
        // No pose yet — open-loop: drive straight
        drive(baseSpeed, baseSpeed);
        return;
    }
    float err   = headingError(myPose.theta, targetTheta);
    float kP    = 1.8f;                    // proportional gain (tune on bench)
    float corr  = kP * err;
    corr        = constrain(corr, -(float)MAX_PWM, (float)MAX_PWM);

    int lDuty = (int)baseSpeed + (int)corr;
    int rDuty = (int)baseSpeed - (int)corr;
    drive(lDuty, rDuty);
}

/*
 * openLoopTurn — spin in place without pose data.
 * dir: +1 = left turn, -1 = right turn.
 */
static void openLoopTurn(float dir, uint8_t speed) {
    int s = (int)speed;
    if (dir > 0.0f) {
        drive(-s, s);   // left motor back, right forward → CCW
    } else {
        drive(s, -s);   // left forward, right back → CW
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wander heading management
// ─────────────────────────────────────────────────────────────────────────────
static void pickNewWanderHeading() {
    // Random heading in [-π, π]
    wanderHeading     = ((float)(random(0, 6284)) / 1000.0f) - (float)M_PI;
    lastHeadingChange = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// Main behaviour update (called every loop iteration)
// ─────────────────────────────────────────────────────────────────────────────
static void updateBehaviour() {
    uint32_t now = millis();

    // ── Global stop from server ───────────────────────────────────────────
    if (!serverRun) {
        if (driveState != DriveState::STOP) {
            stopMotors();
            driveState = DriveState::STOP;
        }
        return;
    }

    // ── Assess threats ────────────────────────────────────────────────────
    float px = myPose.valid ? myPose.x : 0.0f;
    float py = myPose.valid ? myPose.y : 0.0f;

    float edgeClear    = edgeClearance(px, py);
    float escEdgeAngle = nearestEdgeEscapeAngle(px, py);

    float robotEscAngle = wanderHeading;
    float robotDist     = nearestRobotDist(px, py, &robotEscAngle);

    // Determine worst zone we are in
    bool inRobotHard = (robotDist  < ROBOT_HARD_DIST);
    bool inRobotWarn = (robotDist  < ROBOT_WARN_DIST) && !inRobotHard;
    bool inEdgeHard  = (edgeClear  < EDGE_HARD_DIST);
    bool inEdgeWarn  = (edgeClear  < EDGE_WARN_DIST)  && !inEdgeHard;

    bool anyHard     = inRobotHard || inEdgeHard;
    bool anyWarn     = inRobotWarn || inEdgeWarn;

    // Choose the escape heading (edge takes priority over robot if both)
    float escapeAngle = inEdgeHard ? escEdgeAngle
                      : inRobotHard ? robotEscAngle
                      : inEdgeWarn  ? escEdgeAngle
                      : robotEscAngle;

    // ── State transitions ──────────────────────────────────────────────────
    switch (driveState) {

        // ── STOP → WANDER when server sends RUN ───────────────────────────
        case DriveState::STOP:
            pickNewWanderHeading();
            driveState    = DriveState::WANDER;
            stateEnteredAt = now;
            break;

        // ── WANDER ────────────────────────────────────────────────────────
        case DriveState::WANDER:
            if (anyHard) {
                driveState     = DriveState::HARD;
                stateEnteredAt = now;
                // Decide turn direction: try left first, pick the side that
                // aligns best with the escape angle.
                if (myPose.valid) {
                    float errLeft  = headingError(myPose.theta + (float)(M_PI / 4.0), escapeAngle);
                    float errRight = headingError(myPose.theta - (float)(M_PI / 4.0), escapeAngle);
                    avoidDir = (fabsf(errLeft) < fabsf(errRight)) ? 1.0f : -1.0f;
                } else {
                    avoidDir = (random(0, 2) == 0) ? 1.0f : -1.0f;
                }
            } else if (anyWarn) {
                driveState     = DriveState::WARN;
                stateEnteredAt = now;
                if (myPose.valid) {
                    float errLeft  = headingError(myPose.theta + (float)(M_PI / 4.0), escapeAngle);
                    float errRight = headingError(myPose.theta - (float)(M_PI / 4.0), escapeAngle);
                    avoidDir = (fabsf(errLeft) < fabsf(errRight)) ? 1.0f : -1.0f;
                } else {
                    avoidDir = (random(0, 2) == 0) ? 1.0f : -1.0f;
                }
            } else {
                // Normal wander — periodically pick a new heading
                if ((now - lastHeadingChange) > WANDER_CHANGE_MS) {
                    pickNewWanderHeading();
                }
                driveToHeading(wanderHeading, FULL_SPEED);
            }
            break;

        // ── WARN: slow down and steer toward escape angle ─────────────────
        case DriveState::WARN:
            if (anyHard) {
                // Escalate
                driveState     = DriveState::HARD;
                stateEnteredAt = now;
            } else if (!anyWarn) {
                // Threat cleared — resume wander with a fresh heading
                pickNewWanderHeading();
                driveState     = DriveState::WANDER;
                stateEnteredAt = now;
            } else {
                // Steer toward escape at reduced speed
                driveToHeading(escapeAngle, WARN_SPEED);
            }
            break;

        // ── HARD: stop, then reverse-steer ───────────────────────────────
        case DriveState::HARD: {
            uint32_t elapsed = now - stateEnteredAt;

            if (elapsed < REVERSE_DURATION_MS) {
                // Phase 1: back away while turning
                openLoopTurn(avoidDir, REVERSE_SPEED);
            } else if (!anyHard) {
                // Threat cleared — jump to WARN or WANDER as appropriate
                if (anyWarn) {
                    driveState     = DriveState::WARN;
                } else {
                    pickNewWanderHeading();
                    driveState     = DriveState::WANDER;
                }
                stateEnteredAt = now;
            } else {
                // Still in HARD zone — keep reversing
                stateEnteredAt = now;   // reset timer so we keep reacting
                openLoopTurn(avoidDir, REVERSE_SPEED);
            }
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Arduino entry points
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("[Wander] Booting...");

    // Motor direction pins
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);   // enable motor driver

    // PWM channels (LEDC)
    ledcSetup(PWM_CHAN_LEFT,  PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHAN_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWMA, PWM_CHAN_LEFT);
    ledcAttachPin(PIN_PWMB, PWM_CHAN_RIGHT);
    stopMotors();

    // Hardware encoders
    initEncoders();

    // WiFi + UDP
    initWiFi();

    // Seed RNG from floating ADC noise
    randomSeed(analogRead(A0));

    pickNewWanderHeading();
    Serial.println("[Wander] Ready. Waiting for server RUN command...");
}

void loop() {
    // 1. Drain the UDP buffer — grab the latest position broadcast
    pollUDP();

    // 2. Run the wander / avoidance state machine
    updateBehaviour();

    // 3. Small yield so background WiFi tasks can run
    delay(20);   // 50 Hz update rate
}
