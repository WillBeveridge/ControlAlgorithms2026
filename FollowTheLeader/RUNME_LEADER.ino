/*
  RUNME_LEADER.ino
  Robot 1 - drives a circle path uploaded to the server by followLeader.py
  Uses the standard getPath/localize/moveTo flow from RobotControl.h
*/

#include "RobotControl.h"

// Robot ID 1, starting at origin facing up
// Update server address to match your machine's IP
Robot robot(0, 0, pi / 2, 1, "http://192.168.0.101:3000");

void setup()
{
    Serial.begin(9600);
    delay(2000); // wait for ESP8266 to connect to WiFi

    // Get initial localized position from camera
    robot.localize();

    // Tell server this robot is in position and ready
    robot.setReady();

    // Wait for host script to give the go signal
    while (!robot.getReady()) {}
}

void loop()
{
    // Get the circle path chunk from server (only 1 chunk for this test)
    robot.getPath(1);

    int pathLen = robot.pathDoc["path"].size();

    for (int i = 0; i < pathLen; i++)
    {
        float tx = robot.pathDoc["path"][i][0];
        float ty = robot.pathDoc["path"][i][1];

        // Localize every 3 waypoints to correct odometry drift
        if (i % 3 == 0)
        {
            robot.localize();
        }

        robot.moveTo(tx, ty);
        robot.putPosition();
    }
    // Loop() will repeat, running the circle continuously
}
