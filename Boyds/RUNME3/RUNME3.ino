/*
  Code Authored by Keegan Kelly
  Modified: soft-boundary wander + inter-robot collision avoidance for Robot 3

  Robot discovery is automatic — no hardcoded list needed.
  All other robot slots are watched. Any position that is still at the
  server default (0, 0) is ignored — meaning the camera hasn't seen that
  robot so it isn't running.
  
  Only change ROBOT_ID per robot. Everything else is set at runtime.
*/
#include "RobotControl.h"

#define ROBOT_ID   3
#define NUM_ROBOTS 6
String server = "http://192.168.0.101:3000";

// ── Virtual boundary — derived from pixel boundary (SOFT_PX=80, HARD_PX=27)
// Frame 1920x1080, arena 3.60x2.03m, centre = (0,0)
const float X_MIN        = -1.706f;
const float X_MAX        =  1.706f;
const float Y_MIN        = -0.921f;
const float Y_MAX        =  0.921f;
const float OUTER_MARGIN =  0.188f;  // matches green rectangle (80px from edge)
const float INNER_MARGIN =  0.094f;  // matches red rectangle  (27px from edge)

// ── Drive parameters ──────────────────────────────────────────────────────────
const float DRIVE_SPEED     = 0.18f;
const float MAX_W           = 6.0f;
const float MIN_SPEED_SCALE = 0.35f;

// ── Robot-robot collision avoidance ──────────────────────────────────────────
const float ROBOT_SOFT_DIST = 0.40f;
const float ROBOT_HARD_DIST = 0.20f;
const float ROBOT_MAX_W     = 5.0f;

// ── Timing ────────────────────────────────────────────────────────────────────
const unsigned long CAM_POLL_MS   = 200;
const unsigned long ROBOT_POLL_MS = 150;  // one robot fetched per cycle
const unsigned long LOCALIZE_MS   = 5000;
const unsigned long RECOVERY_MS   = 600;

// ─────────────────────────────────────────────────────────────────────────────

Robot robotA(0, 0, pi / 2, ROBOT_ID, server);

// Own position
float camX = 0, camY = 0;
bool  camValid = false;

// All other robot slots — indexed 0 to numActive-1
int   activeIDs[NUM_ROBOTS];
int   numActive = 0;
float otherX[NUM_ROBOTS];
float otherY[NUM_ROBOTS];

// Tracks which robot to fetch next — incremented each poll cycle
// so only ONE robot is fetched per loop iteration
int   nextRobotIdx = 0;

// ── Build list of all other robot IDs ────────────────────────────────────────
// No server check needed — just populate every ID except our own.
// Positions start at 9999 so they don't trigger repulsion before first fetch.
void discoverActiveRobots()
{
    numActive = 0;
    for (int id = 1; id <= NUM_ROBOTS; id++)
    {
        if (id == ROBOT_ID) continue;
        activeIDs[numActive] = id;
        otherX[numActive]    = 9999;
        otherY[numActive]    = 9999;
        numActive++;
    }
}

// ── Quick own position update ─────────────────────────────────────────────────
bool quickCamUpdate()
{
    while (Serial.available()) Serial.read();

    char address[35];
    strcpy(address, robotA.serverAddress);
    strcat(address, "/agents/");
    char idChar[2] = {ROBOT_ID + '0', '\0'};
    strcat(address, idChar);

    StaticJsonDocument<100> req;
    req["type"]    = "GET";
    req["address"] = address;
    serializeJson(req, Serial);
    req.clear();

    unsigned long t0 = millis();
    while (millis() - t0 < 60)
    {
        if (Serial.available())
        {
            StaticJsonDocument<130> resp;
            if (deserializeJson(resp, Serial) == DeserializationError::Ok
                && resp.containsKey("position"))
            {
                camX     = resp["position"][0].as<float>();
                camY     = resp["position"][1].as<float>();
                camValid = true;
                return true;
            }
            return false;
        }
    }
    return false;
}

// ── Fetch ONE other robot's position per call ─────────────────────────────────
// Rotates through all other robots one at a time using nextRobotIdx.
// Max block time is 30ms per call — drive() is never starved.
void fetchNextRobot()
{
    if (numActive == 0) return;

    while (Serial.available()) Serial.read();

    int i = nextRobotIdx % numActive;
    nextRobotIdx++;

    char address[35];
    strcpy(address, robotA.serverAddress);
    strcat(address, "/agents/");
    char idChar[2] = {activeIDs[i] + '0', '\0'};
    strcat(address, idChar);

    StaticJsonDocument<100> req;
    req["type"]    = "GET";
    req["address"] = address;
    serializeJson(req, Serial);
    req.clear();

    unsigned long t0 = millis();
    while (millis() - t0 < 30)
    {
        if (Serial.available())
        {
            StaticJsonDocument<130> resp;
            if (deserializeJson(resp, Serial) == DeserializationError::Ok
                && resp.containsKey("position"))
            {
                float fx = resp["position"][0].as<float>();
                float fy = resp["position"][1].as<float>();
                // (0, 0) is the server default — robot not seen, skip it
                if (!(fx == 0.0f && fy == 0.0f))
                {
                    otherX[i] = fx;
                    otherY[i] = fy;
                }
            }
            return;
        }
    }
    // timed out — keep last known value
}

// ── Boundary inner zone check ─────────────────────────────────────────────────
bool inInnerZone(float px, float py)
{
    return (px < X_MIN + INNER_MARGIN ||
            px > X_MAX - INNER_MARGIN ||
            py < Y_MIN + INNER_MARGIN ||
            py > Y_MAX - INNER_MARGIN);
}

// ── Robot too close check ─────────────────────────────────────────────────────
bool robotTooClose(float px, float py)
{
    for (int i = 0; i < numActive; i++)
    {
        float dx = px - otherX[i];
        float dy = py - otherY[i];
        if (sqrt(dx*dx + dy*dy) < ROBOT_HARD_DIST) return true;
    }
    return false;
}

// ── Boundary repulsion ────────────────────────────────────────────────────────
float computeBoundaryCorrection(float px, float py, float heading, float &speedScale)
{
    float zoneWidth = OUTER_MARGIN - INNER_MARGIN;
    auto depth = [&](float d) -> float {
        if (d <= 0) return 0.0f;
        return min(d / zoneWidth, 1.0f);
    };
    float dL = depth((X_MIN + OUTER_MARGIN) - px);
    float dR = depth(px - (X_MAX - OUTER_MARGIN));
    float dB = depth((Y_MIN + OUTER_MARGIN) - py);
    float dT = depth(py - (Y_MAX - OUTER_MARGIN));
    float maxDepth = max(max(dL, dR), max(dB, dT));
    speedScale = max(1.0f - maxDepth, MIN_SPEED_SCALE);
    if (maxDepth == 0.0f) return 0.0f;
    float repX = dL - dR;
    float repY = dB - dT;
    float w    = (repX * (-sin(heading)) + repY * cos(heading)) * MAX_W;
    return constrain(w, -MAX_W, MAX_W);
}

// ── Robot-robot repulsion ─────────────────────────────────────────────────────
float computeRobotRepulsion(float px, float py, float heading, float &robotSpeedScale)
{
    float repX = 0, repY = 0;
    float minDist = 9999;
    robotSpeedScale = 1.0f;

    for (int i = 0; i < numActive; i++)
    {
        float dx   = px - otherX[i];
        float dy   = py - otherY[i];
        float dist = sqrt(dx*dx + dy*dy);
        if (dist < ROBOT_SOFT_DIST && dist > 0.001f)
        {
            float zoneWidth = ROBOT_SOFT_DIST - ROBOT_HARD_DIST;
            float d = min((ROBOT_SOFT_DIST - dist) / zoneWidth, 1.0f);
            repX += (dx / dist) * d;
            repY += (dy / dist) * d;
            if (dist < minDist) minDist = dist;
        }
    }
    if (minDist < ROBOT_SOFT_DIST)
    {
        float zoneWidth = ROBOT_SOFT_DIST - ROBOT_HARD_DIST;
        float depth     = min((ROBOT_SOFT_DIST - minDist) / zoneWidth, 1.0f);
        robotSpeedScale = max(1.0f - depth, MIN_SPEED_SCALE);
    }
    if (repX == 0 && repY == 0) return 0.0f;
    float w = (repX * (-sin(heading)) + repY * cos(heading)) * ROBOT_MAX_W;
    return constrain(w, -ROBOT_MAX_W, ROBOT_MAX_W);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup(void)
{
    Serial.begin(115200);
    delay(2000);

    int ok = 0;
    while (!ok) { ok = robotA.localize(); }
    camX = robotA.x;
    camY = robotA.y;
    camValid = true;

    robotA.setReady();
    int ready = 0;
    while (!ready) { ready = robotA.getReady(); }

    // Populate all other robot IDs — no server check needed
    discoverActiveRobots();
}

void loop(void)
{
    unsigned long lastCamPoll   = millis();
    unsigned long lastRobotPoll = millis();
    unsigned long lastLocalize  = millis();

    while (true)
    {
        robotA.updatePositionPublic();
        unsigned long now = millis();

        // ── Full localize ─────────────────────────────────────────────────
        if (now - lastLocalize > LOCALIZE_MS)
        {
            robotA.drive(0, 0);
            robotA.localize();
            camX         = robotA.x;
            camY         = robotA.y;
            camValid     = true;
            lastLocalize = now;
            lastCamPoll  = now;
        }
        else if (now - lastCamPoll > CAM_POLL_MS)
        {
            quickCamUpdate();
            lastCamPoll = now;
        }

        if (now - lastRobotPoll > ROBOT_POLL_MS)
        {
            fetchNextRobot();
            lastRobotPoll = now;
        }

        float px = camValid ? camX : robotA.x;
        float py = camValid ? camY : robotA.y;

        // ── Recovery: boundary ────────────────────────────────────────────
        if (inInnerZone(px, py))
        {
            unsigned long recStart = millis();
            while (millis() - recStart < RECOVERY_MS)
            {
                robotA.updatePositionPublic();
                robotA.drive(-DRIVE_SPEED, 0);
            }
            robotA.localize();
            camX = robotA.x; camY = robotA.y;
            camValid = true;
            lastLocalize = lastCamPoll = millis();
            continue;
        }

        // ── Recovery: robot too close ─────────────────────────────────────
        if (robotTooClose(px, py))
        {
            unsigned long recStart = millis();
            while (millis() - recStart < RECOVERY_MS)
            {
                robotA.updatePositionPublic();
                robotA.drive(-DRIVE_SPEED, 0);
            }
            robotA.localize();
            camX = robotA.x; camY = robotA.y;
            camValid = true;
            lastLocalize = lastCamPoll = millis();
            continue;
        }

        // ── Combine boundary + robot repulsion and drive ──────────────────
        float boundaryScale = 1.0f, robotScale = 1.0f;
        float wBoundary = computeBoundaryCorrection(px, py, robotA.theta, boundaryScale);
        float wRobot    = computeRobotRepulsion(px, py, robotA.theta, robotScale);

        float speedScale = min(boundaryScale, robotScale);
        float w          = constrain(wBoundary + wRobot, -MAX_W, MAX_W);

        robotA.drive(DRIVE_SPEED * speedScale, w);
    }
}
