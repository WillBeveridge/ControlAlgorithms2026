/*
Code Authored by Keegan Kelly
Modified: removed delay at end of loop so the robot immediately waits
for the next command from goto.py rather than sitting idle for 500 seconds.
*/

#include "RobotControl.h"

// ── Change this ID for each robot (1–6) ──────────────────────────────────────
#define id 6
// ─────────────────────────────────────────────────────────────────────────────

String server = "http://192.168.0.100:3000";

void setup(void)
{
  Serial.begin(115200);
  delay(2000);
}

Robot robotA(0, 0, pi / 2, id, server);

void loop(void)
{
  // Fetch this robot's path from /goal{id}/1
  // goto.py always writes the new path BEFORE signalling ready,
  // so by the time we get here the path is already up to date.
  robotA.getPath(1);
  int idx   = robotA.pathDoc["id"].as<int>();
  int total = robotA.pathDoc["total"].as<int>();

  // Drive to the staging position (path[0]).
  // goto.py sets path[0] to the robot's current position, so this is instant.
  robotA.localize();
  robotA.moveTo(
    robotA.pathDoc["path"][0][0].as<float>(),
    robotA.pathDoc["path"][0][1].as<float>()
  );

  // Signal that we are in position and waiting
  robotA.setReady();

  // Wait for goto.py to confirm the path is loaded and fire the go signal
  int Ready = 0;
  while (!Ready)
  {
    Ready = robotA.getReady();
  }

  // Execute all path segments
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

  // Path complete — loop back immediately.
  // goto.py will reset agentReady and agentGo before writing the next path,
  // so the robot won't accidentally re-trigger on stale signals.
}
