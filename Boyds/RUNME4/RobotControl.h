/*
Code Authored by Keegan Kelly
Modified: made serverAddress public, added updatePositionPublic()
          for use by RUNME3's continuous drive loop.
          All calibration values unchanged from original.
*/
#include <math.h>
#include <ArduinoJson.h>
// Pin definitions
#define DIRR 7  // Direction control for motor B
#define DIRL 8  // Direction control for motor A
#define PWML 9  // PWM control (speed) for motor A
#define PWMR 10 // PWM control (speed) for motor B
#define leftEncoder 2
#define rightEncoder 3
unsigned int leftEncoderTicks = 0;
unsigned int rightEncoderTicks = 0;
float pi = M_PI;
void incrementLeftEncoder()  { leftEncoderTicks++;  }
void incrementRightEncoder() { rightEncoderTicks++; }
void clearLeftEncoder()      { leftEncoderTicks  = 0; }
void clearRightEncoder()     { rightEncoderTicks = 0; }
float fixAngle(float angle)
{
    while (angle >  pi) angle -= 2.0f * pi;
    while (angle < -pi) angle += 2.0f * pi;
    return angle;
}
class Robot
{
public:
    float x, y, theta;
    int id;
    // Made public so RUNME3 can build request addresses (e.g. /agents/<id>)
    char serverAddress[30] = {};
    StaticJsonDocument<450> pathDoc;

    Robot(float X, float Y, float THETA, int ID, String address)
    {
        x = X; y = Y; theta = THETA; id = ID;
        address.toCharArray(serverAddress, 30);
        setupArdumoto();
        pinMode(leftEncoder,  INPUT_PULLUP);
        pinMode(rightEncoder, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(leftEncoder),  incrementLeftEncoder,  CHANGE);
        attachInterrupt(digitalPinToInterrupt(rightEncoder), incrementRightEncoder, CHANGE);
    }

    void moveTo(float X, float Y)
    {
        float integral = 0, derivative = 0;
        float integralTheta = 0, derivativeTheta = 0;
        float prevDirectionalErr = 0, prevThetaErr = 0;
        int prevTime = micros(), currentTime;
        float directionalErr;
        float err      = sqrt(pow(X - x, 2) + pow(Y - y, 2));
        float thetaErr = fixAngle(atan2(Y - y, X - x) - theta);
        while (err > Err)
        {
            updatePosition();
            currentTime    = micros();
            err            = sqrt(pow(X - x, 2) + pow(Y - y, 2));
            thetaErr       = atan2(Y - y, X - x) - theta;
            directionalErr = err * cos(thetaErr);
            if (directionalErr < 0.0f)
                thetaErr = fixAngle(thetaErr - pi);
            else
                thetaErr = fixAngle(thetaErr);
            derivative      = (directionalErr - prevDirectionalErr) / (currentTime - prevTime) * 1000000;
            integral       += directionalErr * (currentTime - prevTime) / 1000000;
            integralTheta  += thetaErr       * (currentTime - prevTime) / 1000000;
            derivativeTheta = (thetaErr - prevThetaErr) / (currentTime - prevTime) * 1000000;
            prevTime           = currentTime;
            prevDirectionalErr = directionalErr;
            prevThetaErr       = thetaErr;
            float v = 2.0f * Kp * directionalErr + Ki * integral    + Kd * derivative;
            float w = KpTheta * thetaErr + KiTheta * integralTheta  + KdTheta * derivativeTheta;
            drive(v, w);
        }
        drive(0, 0);
    }

    void drive(float v, float w)
    {
        float wR = (v + WB / 2 * w) / r;
        float wL = (v - WB / 2 * w) / r;
        if (wR > 0) { DirWR = 1; } else { DirWR = 0; }
        if (wL > 0) { DirWL = 1; } else { DirWL = 0; }
        int WR = map(abs(wR), 0, 19.5, 90, 255);
        int WL = map(abs(wL), 0, 19.5, 90, 255);
        if      (WR > 255)        WR = 255;
        else if (abs(wR) <= 0.35) WR = 0;
        if      (WL > 255)        WL = 255;
        else if (abs(wL) <= 0.35) WL = 0;
        digitalWrite(DIRR, DirWR);
        digitalWrite(DIRL, DirWL);
        analogWrite(PWMR, WR);
        analogWrite(PWML, WL);
    }

    // Public wrapper so RUNME3's main loop can tick odometry each iteration
    void updatePositionPublic() { updatePosition(); }

    int localize()
    {
        char address[35];
        strcpy(address, serverAddress);
        strcat(address, "/agents/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<130> req;
        req["type"]    = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        req.clear();
        while (1)
        {
            if (Serial.available())
            {
                DeserializationError error = deserializeJson(req, Serial);
                if (error != DeserializationError::Ok) return 0;
                x     = req["position"][0];
                y     = req["position"][1];
                theta = req["position"][2];
                req.clear();
                clearLeftEncoder();
                clearRightEncoder();
                return 1;
            }
        }
    }

    void getPath(int idx)
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/goal");
        char tempChar[4] = {id + '0', '/', idx + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<90> req;
        req["type"]    = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        pathDoc.clear();
        while (1)
        {
            if (Serial.available())
            {
                DeserializationError error = deserializeJson(pathDoc, Serial);
                if (error != DeserializationError::Ok) { getPath(idx); return; }
                else return;
            }
        }
    }

    void putPosition()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentsLocal/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<80> req;
        req["type"]    = "PUT";
        req["address"] = address;
        req["id"]      = id;
        JsonArray position = req.createNestedArray("position");
        position.add(x); position.add(y); position.add(theta);
        serializeJson(req, Serial);
        req.clear();
    }

    void setReady()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentReady/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<80> req;
        req["type"]  = "PUT";
        req["address"] = address;
        req["id"]    = id;
        req["ready"] = 1;
        serializeJson(req, Serial);
        req.clear();
    }

    int getReady()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentGo/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<80> req;
        req["type"]    = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        req.clear();
        while (1)
        {
            if (Serial.available())
            {
                DeserializationError error = deserializeJson(req, Serial);
                if (error != DeserializationError::Ok) return 0;
                else if (req["ready"] == 1) { req.clear(); return 1; }
                else                        { req.clear(); return 0; }
            }
        }
    }

private:
    // ── Calibration values — unchanged from original ──────────────────────
    float r  = 0.033 * 1.2;
    float WB = 0.164 * 0.75f;
    float CL = pi / 384.0f * r;
    float CR = pi / 384.0f * r;
    uint8_t DirWL = 1, DirWR = 1;
    float Kp = 1.7,  Ki = 0.005, Kd = 0.5;
    float KpTheta = 11, KiTheta = 1, KdTheta = 1.5;
    float Err = 0.03;

    void updatePosition()
    {
        int diffLeft  = leftEncoderTicks;  clearLeftEncoder();
        int diffRight = rightEncoderTicks; clearRightEncoder();
        float dTheta = 0, dL = 0;
        if (DirWL) { dTheta -= diffLeft  * CL / WB * 2; dL += diffLeft  * CL; }
        else       { dTheta += diffLeft  * CL / WB * 2; dL -= diffLeft  * CL; }
        if (DirWR) { dTheta += diffRight * CR / WB * 2; dL += diffRight * CR; }
        else       { dTheta -= diffRight * CR / WB * 2; dL -= diffRight * CR; }
        float EncoderdX = dL * cos(theta + 0.5f * dTheta);
        float EncoderdY = dL * sin(theta + 0.5f * dTheta);
        theta += dTheta;
        fixTheta();
        x += EncoderdX;
        y += EncoderdY;
    }

    void fixTheta()
    {
        while (theta >  pi) theta -= 2.0f * pi;
        while (theta < -pi) theta += 2.0f * pi;
    }

    void setupArdumoto()
    {
        pinMode(PWML, OUTPUT); pinMode(PWMR, OUTPUT);
        pinMode(DIRL, OUTPUT); pinMode(DIRR, OUTPUT);
        digitalWrite(PWML, LOW); digitalWrite(PWMR, LOW);
        digitalWrite(DIRL, LOW); digitalWrite(DIRR, LOW);
    }
};
