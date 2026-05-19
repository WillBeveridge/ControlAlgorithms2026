/*
Code Authored by Keegan Kelly
Modified for manual wireless control with camera localization
*/

#include "RobotControl.h"
#define id 4
String server = "http://192.168.0.102:3000";

void setup(void)
{
  Serial.begin(115200);
  delay(2000);
}

Robot robotA(0, 0, pi / 2, id, server);

void loop(void)
{
  // Wait for go signal from the laptop
  int Ready = 0;
  while (!Ready)
  {
    delay(500);
    Ready = robotA.getReady();
  }

  // Fetch path from server
  robotA.getPath(1);
  int len = robotA.pathDoc["path"].size();
  float dt = robotA.pathDoc["dt"].as<float>();
  int update = robotA.pathDoc["update"].as<int>();

  // Execute each waypoint in the path
  for (int i = 0; i < len; i++)
  {
    float prevTime = millis();
    // localize every 'update' steps using camera position
    if (i % update == 0)
    {
      int success = 0;
      while (!success)
      {
        success = robotA.localize();
      }
    }
    robotA.moveTo(robotA.pathDoc["path"][i][0].as<float>(), robotA.pathDoc["path"][i][1].as<float>());
    while ((millis() - prevTime) * 0.001 < dt) {}
  }

  // Signal done and loop back for next command
  robotA.setReady();
}
