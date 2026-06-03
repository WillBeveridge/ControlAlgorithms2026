/*
Code Authored by Keegan Kelly
Modified: removed delay at end of loop so the robot immediately waits
for the next command from goto.py rather than sitting idle for 500 seconds.
*/

#include "RobotControl.h"

// ── Change this ID for each robot (1–6) ──────────────────────────────────────
#define id 4
// ─────────────────────────────────────────────────────────────────────────────

String server = "http://192.168.0.101:3000";

void setup(void)
{
  Serial.begin(115200);
  delay(2000);
}

Robot robotA(0, 0, pi / 2, id, server);

void loop(void)
{
  robotA.setReady();

  // Get real starting position before waiting for go
  int success = 0;
  while (!success)
  {
    success = robotA.localize();
  }

  int Ready = 0;
  while (!Ready)
  {
    Ready = robotA.getReady();
  }

  robotA.getPath(1);
  int idx   = robotA.pathDoc["id"].as<int>();
  int total = robotA.pathDoc["total"].as<int>();

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

      // Localize every `update` waypoints to correct drift
      if (i % update == 0)
      {
        int success = 0;
        while (!success)
        {
          success = robotA.localize();
        }
      }

      robotA.moveTo(
        robotA.pathDoc["path"][i][0].as<float>(),
        robotA.pathDoc["path"][i][1].as<float>()
      );

      // Hold until the next time step
      while ((millis() - prevTime) * 0.001 < robotA.pathDoc["dt"].as<float>())
      {
        // wait
      }
    }
    idx++;
  }

  // Hold at final position until goto.py sends a new go command
  float finalX = robotA.pathDoc["path"][robotA.pathDoc["path"].size()-1][0].as<float>();
  float finalY = robotA.pathDoc["path"][robotA.pathDoc["path"].size()-1][1].as<float>();
  while (!robotA.getReady())
  {
    robotA.moveTo(finalX, finalY);
    delay(500);
  }
  // Path complete — loop back immediately.
  // goto.py will reset agentReady and agentGo before writing the next path,
  // so the robot won't accidentally re-trigger on stale signals.
}
