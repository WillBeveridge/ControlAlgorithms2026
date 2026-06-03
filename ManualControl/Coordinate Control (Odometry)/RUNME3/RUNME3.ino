/*
Code Authored by Keegan Kelly
Modified: robot now waits for goto.py to be ready before doing anything,
preventing it from executing stale paths on boot.
*/

#include "RobotControl.h"

// ── Change this ID for each robot (1–6) ──────────────────────────────────────
#define id 4
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
  // Signal to goto.py that we are booted and waiting for a path
  robotA.setReady();

  // Wait for goto.py to write the new path and fire the go signal
  int Ready = 0;
  while (!Ready)
  {
    Ready = robotA.getReady();
  }

  // Fetch the path that goto.py just wrote
  robotA.getPath(1);
  int idx   = robotA.pathDoc["id"].as<int>();
  int total = robotA.pathDoc["total"].as<int>();

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

  // Path complete — loop back and wait for the next command from goto.py
}
