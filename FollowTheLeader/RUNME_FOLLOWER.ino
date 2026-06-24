/*
  RUNME_FOLLOWER.ino
  Robot 2 - continuously reads Robot 1's camera-localized position
  from the server and drives to it.
  Uses the same serial JSON pattern as RobotControl.h to talk to the ESP8266.
*/

#include "RobotControl.h"

// Robot ID 2, starting at origin facing up
// Update server address to match your machine's IP
Robot robot(0, 0, pi / 2, 2, "http://192.168.0.101:3000");

// Fetches the current camera position of robot with given id from /agents/<targetId>
// Returns true on success and sets tx, ty to the target position
bool getLeaderPos(float &tx, float &ty)
{
    char address[40];
    strcpy(address, robot.serverAddress);
    strcat(address, "/agents/1");

    StaticJsonDocument<130> req;
    req["type"] = "GET";
    req["address"] = address;
    serializeJson(req, Serial);
    req.clear();

    // Wait for ESP8266 to return the response
    unsigned long timeout = millis() + 3000;
    while (!Serial.available())
    {
        if (millis() > timeout) return false;
    }

    StaticJsonDocument<130> resp;
    DeserializationError err = deserializeJson(resp, Serial);
    if (err != DeserializationError::Ok) return false;

    tx = resp["position"][0];
    ty = resp["position"][1];
    return true;
}

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
    float tx, ty;

    // Re-localize every loop to keep follower accurate
    robot.localize();

    // Fetch leader's current position from server
    if (getLeaderPos(tx, ty))
    {
        robot.moveTo(tx, ty);
        robot.putPosition();
    }
}
