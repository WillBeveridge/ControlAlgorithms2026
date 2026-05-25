/*
Code Authored by Keegan Kelly
Modified for manual_control.py compatibility:
  - Added "wait for agentGo == 0" gate at the top of loop() so the robot
    holds after finishing a path until the server has been reset with a new
    command.  This prevents the robot from grabbing a stale path chunk.
  - Removed delay(500000) — the robot now loops back immediately and waits
    at the gate instead of sitting idle for ~8 minutes.
*/
#include "RobotControl.h"
#define id 3
String server = "http://192.168.0.101:3000";

void setup(void)
{
  Serial.begin(115200);
  delay(2000);  // wait for ESP8266 to connect to WiFi
}

Robot robotA(0, 0, pi / 2, id, server);

void loop(void)
{
  // ── Gate: wait here until the server signals agentGo == 0 ────────────────
  // manual_control.py sets agentGo to 0 (via prepare_server_for_move) AFTER
  // uploading the new path, so by the time this unblocks the new path is
  // already on the server.
  // On first boot agentGo starts at 0 so this exits immediately.
  while (robotA.getReady())
  {
    delay(100);
  }
  // ─────────────────────────────────────────────────────────────────────────

  robotA.getPath(1);
  int idx   = 1;
  int total = robotA.pathDoc["total"].as<int>();

  robotA.localize();
  robotA.moveTo(robotA.pathDoc["path"][0][0].as<float>(),
                robotA.pathDoc["path"][0][1].as<float>());
  robotA.setReady();

  int Ready = 0;
  while (!Ready)
  {
    Ready = robotA.getReady();
  }

  while (idx <= total)
  {
    if (idx != 1)
    {
      robotA.getPath(idx);
    }
    int len    = robotA.pathDoc["path"].size();
    int update = robotA.pathDoc["update"].as<int>();

    for (int i = 0; i < len; i++)
    {
      float prevTime = millis();
      int   success  = 0;

      if (i % update == 0)
      {
        while (!success)
        {
          success = robotA.localize();
        }
      }

      robotA.moveTo(robotA.pathDoc["path"][i][0].as<float>(),
                    robotA.pathDoc["path"][i][1].as<float>());

      while ((millis() - prevTime) * 0.001 < robotA.pathDoc["dt"].as<float>())
      {
        // wait for next time step
      }
    }
    idx++;
  }
  // No delay here — loop() restarts and the gate above holds until
  // manual_control.py uploads a new path and resets agentGo to 0.
}
