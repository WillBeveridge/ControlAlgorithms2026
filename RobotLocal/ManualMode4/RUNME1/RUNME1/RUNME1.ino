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
  // Fetch drive command from server and apply it to motors
  robotA.getDriveCmd();
  delay(100); // poll at 10Hz
}