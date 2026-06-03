/*
Code Authored by Keegan Kelly
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
// Encoder ticks and interrupt functions are global because it wouldn't compile with them as members and methods
unsigned int leftEncoderTicks = 0;
unsigned int rightEncoderTicks = 0;
float pi = M_PI;
// Interrupts for encoders
void incrementLeftEncoder()
{
    leftEncoderTicks++;
}
void incrementRightEncoder()
{
    rightEncoderTicks++;
}
// Clears the encoder ticks to prevent overflow
void clearLeftEncoder()
{
    leftEncoderTicks = 0;
}
void clearRightEncoder()
{
    rightEncoderTicks = 0;
}
// same as fix theta but it changes a given angle instead of the robot heading
float fixAngle(float angle)
{
    while (angle > pi)
    {
        angle -= 2.0f * pi;
    }
    while (angle < -pi)
    {
        angle += 2.0f * pi;
    }
    return angle;
}
// Robot class for controlling the robot
class Robot
{
public:
    // position of the robot on the grid
    float x, y, theta;
    // robot id
    int id;
    // address of the server
    char serverAddress[30] = {};
    StaticJsonDocument<450> pathDoc;
    // sets up required pin modes and objects
    Robot(float X, float Y, float THETA, int ID, String address)
    {
        // sets the position and heading to the specified values
        x = X;
        y = Y;
        theta = THETA;
        id = ID;

        address.toCharArray(serverAddress, 30);
        // initializes the motor controller
        setupArdumoto();
        // initializes the encoders and interrupts
        pinMode(leftEncoder, INPUT_PULLUP);
        pinMode(rightEncoder, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(leftEncoder), incrementLeftEncoder, CHANGE);
        attachInterrupt(digitalPinToInterrupt(rightEncoder), incrementRightEncoder, CHANGE);
    }
    // send x,y position and the robot will move to that position
    void moveTo(float X, float Y)
    {
        // variables for the PID controller
        float integral = 0;
        float derivative = 0;
        float integralTheta = 0;
        float derivativeTheta = 0;
        float prevDirectionalErr;
        float prevThetaErr;
        int prevTime = micros();
        int currentTime;
        float directionalErr;

        // absolute positional error
        float err = sqrt(pow(X - x, 2) + pow(Y - y, 2));
        // angle error
        float thetaErr = fixAngle(atan2(Y - y, X - x) - theta);
        // used for debugging
        // putPosition();
        //  continues to drive while the absolute positional Error on position is greater than the tolerance of Err
        while (err > Err)
        {
            // updates position, time, and error
            // putPosition();
            updatePosition();
            currentTime = micros();
            err = sqrt(pow(X - x, 2) + pow(Y - y, 2));
            // no need to fix the error angle here because it does not need to be in the bounds for the cos function and it will be fixed afterwards
            thetaErr = atan2(Y - y, X - x) - theta;
            // multiplying by the cosine of the angle error to return directionality to the
            // absolute error (this will also cause the robot to drive in reverse and set the speed
            // according to how productive moving forward or backward is)
            directionalErr = err * cos(thetaErr);
            // flips the error by 180 degrees if the robot is driving backwards
            // angle is fixed here so it is only done once per iteration
            if (directionalErr < 0.0f)
            {
                thetaErr = fixAngle(thetaErr - pi);
            }
            else
            {
                thetaErr = fixAngle(thetaErr);
            }
            // PID control for the linear and angular velocity
            derivative = (directionalErr - prevDirectionalErr) / (currentTime - prevTime) * 1000000;
            integral += directionalErr * (currentTime - prevTime) / 1000000;
            integralTheta += thetaErr * (currentTime - prevTime) / 1000000;
            derivativeTheta = (thetaErr - prevThetaErr) / (currentTime - prevTime) * 1000000;
            prevTime = currentTime;
            prevDirectionalErr = directionalErr;
            prevThetaErr = thetaErr;
            // sets the linear and angular velocity
            // modifier constants are added to adjust total speeds because the gains are tuned relative to eachother and the constant will just adjust the total speed (if you increase the linear velocity angular does not need to be adjusted by the same amount since they are independent but if its moving a lot faster it might need to turn a bit more aggressively)
            float v = 1.7f * Kp * directionalErr + Ki * integral + Kd * derivative;
            float w = KpTheta * thetaErr + KiTheta * integralTheta + KdTheta * derivativeTheta;
            // sets the speed of the robot
            drive(v, w);
        }
        // set the speed to zero once the new position is reached
        drive(0, 0);
    }

    // moves the robot at velocity v and angular velocity w
    void drive(float v, float w)
    {
        // angular velocities
        float wR = (v + WB / 2 * w) / r;
        float wL = (v - WB / 2 * w) / r;
        // set Directions according to the speed
        // 1 moves forward, 0 moves backward (if this isn't true on a robot flip the wires going to the motor)
        if (wR > 0)
        {
            DirWR = 1;
        }
        else
        {
            DirWR = 0;
        }
        if (wL > 0)
        {
            DirWL = 1;
        }
        else
        {
            DirWL = 0;
        }
        // using the calibrated values convert the speed to the correct PWM values
        // at max reading W of each wheel is about 19.5 on the floor(precision does not really matter its just helpful)
        // set the max left wheel speed a little higher to account for differences in the motors to make it drive straighter
        int WR = map(abs(wR), 0, 19.5, 90, 255);
        int WL = map(abs(wL), 0, 19.5, 90, 255);
        // setting cutoffs for the motors
        if (WR > 255)
            WR = 255;
        else if (abs(wR) <= 0.35)
            WR = 0;
        if (WL > 255)
            WL = 255;
        else if (abs(wL) <= 0.35)
            WL = 0;
        // sending signal to the motors
        digitalWrite(DIRR, DirWR);
        digitalWrite(DIRL, DirWL);
        analogWrite(PWMR, WR);
        analogWrite(PWML, WL);
    }

    // Gets current x,y,theta from the server and updates the robot's position
    int localize()
    {
        char address[35];
        strcpy(address, serverAddress);
        strcat(address, "/agents/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        // sends the address of the get request to the ESP8266
        // doc size determined by this int BUFFER_SIZE = JSON_OBJECT_SIZE(2) + JSON_ARRAY_SIZE(0); where object is the number of key value pairs and array size is the number of elements in the array
        StaticJsonDocument<130> req;
        req["type"] = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        req.clear();
        // waits for the ESP8266 to send the data
        while (1)
        {
            if (Serial.available())
            {
                // loads the data into the json document
                DeserializationError error = deserializeJson(req, Serial);
                // if the data is not valid try again until it is
                if (error != DeserializationError::Ok)
                {
                    return 0;
                }
                else
                {
                    x = req["position"][0];
                    y = req["position"][1];
                    theta = req["position"][2];
                    req.clear();
                    clearLeftEncoder();
                    clearRightEncoder();
                    return 1;
                }
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
        // sends the address of the get request to the ESP8266
        // doc size determined by this int BUFFER_SIZE = JSON_OBJECT_SIZE(2) + JSON_ARRAY_SIZE(0); where object is the number of key value pairs and array size is the number of elements in the array
        StaticJsonDocument<90> req;
        req["type"] = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        pathDoc.clear();
        // waits for the ESP8266 to send the data
        while (1)
        {
            if (Serial.available())
            {
                // loads the data into the json document
                DeserializationError error = deserializeJson(pathDoc, Serial);
                // if the data is not valid try again until it is
                if (error != DeserializationError::Ok)
                {
                    getPath(idx);
                    return;
                }
                else
                {
                    return;
                }
            }
        }
    }

    // sends the current x,y,theta odometry estimate to the server
    void putPosition()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentsLocal/");
        char tempChar[2] = {id + '0', '\0'};

        // sends the address of the get request to the ESP8266
        StaticJsonDocument<80> req;

        req["type"] = "PUT";
        req["address"] = address;
        req["id"] = id;
        JsonArray position = req.createNestedArray("position");
        position.add(x);
        position.add(y);
        position.add(theta);
        serializeJson(req, Serial);
        req.clear();
    }
    // sets the agents ready status to 1 on the server
    void setReady()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentReady/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<80> req;
        req["type"] = "PUT";
        req["address"] = address;
        req["id"] = id;
        req["ready"] = 1;
        serializeJson(req, Serial);
        req.clear();
    }
    // checks if the system wants the agent to go
    int getReady()
    {
        char address[40];
        strcpy(address, serverAddress);
        strcat(address, "/agentGo/");
        char tempChar[2] = {id + '0', '\0'};
        strcat(address, tempChar);
        StaticJsonDocument<80> req;
        req["type"] = "GET";
        req["address"] = address;
        serializeJson(req, Serial);
        req.clear();
        // waits for the ESP8266 to send the data
        while (1)
        {
            if (Serial.available())
            {
                // loads the data into the json document
                DeserializationError error = deserializeJson(req, Serial);
                // if the data is not valid try again until it is
                if (error != DeserializationError::Ok)
                {
                    return 0;
                }
                else if (req["ready"] == 1)
                {
                    req.clear();
                    return 1;
                }
                else
                {
                    req.clear();
                    return 0;
                }
            }
        }
    }

private:
    // radius of the wheels
    float r = 0.045;
    // wheel base of the robot
    float WB = 0.164 * 1.05f;
    // calibration factor to convert encoder ticks to distance
    float CL = pi / 384.0f * r;
    float CR = pi / 384.0f * r;
    // direction of each wheel, 1 is forward, 0 is backward
    uint8_t DirWL = 1;
    uint8_t DirWR = 1;
    // Linear velocity gains
    float Kp = 1.7;
    float Ki = 0.005;
    float Kd = 0.5;
    // Angular velocity gains
    float KpTheta = 11;
    float KiTheta = 1;
    float KdTheta = 1.5;
    // Acceptable position error
    float Err = 0.03;

    // updates the position estimate of the robot based on the encoders
    void updatePosition()
    {
        // encoder ticks since last update
        int diffLeft = leftEncoderTicks;
        clearLeftEncoder();
        int diffRight = rightEncoderTicks;
        clearRightEncoder();
        // angle change since last update
        float dTheta = 0;
        float dL = 0;
        if (DirWL)
        {
            dTheta -= diffLeft * CL / WB * 2;
            dL += diffLeft * CL;
        }
        else
        {
            dTheta += diffLeft * CL / WB * 2;
            dL -= diffLeft * CL;
        }
        if (DirWR)
        {
            dTheta += diffRight * CR / WB * 2;
            dL += diffRight * CR;
        }
        else
        {
            dTheta -= diffRight * CR / WB * 2;
            dL -= diffRight * CR;
        }

        // calculating the new position
        float EncoderdX = dL * cos(theta + 0.5f * dTheta);
        float EncoderdY = dL * sin(theta + 0.5f * dTheta);
        theta += (dTheta);
        fixTheta();
        x += EncoderdX;
        y += EncoderdY;
    }

    // returns theta to the bounds +/-pi
    void fixTheta()
    {
        while (theta > pi)
        {
            theta -= 2.0f * pi;
        }
        while (theta < -pi)
        {
            theta += 2.0f * pi;
        }
    }

    // sets up the motor controller
    void setupArdumoto()
    {
        // All pins should be setup as outputs:
        pinMode(PWML, OUTPUT);
        pinMode(PWMR, OUTPUT);
        pinMode(DIRL, OUTPUT);
        pinMode(DIRR, OUTPUT);

        // Initialize all pins as low:
        digitalWrite(PWML, LOW);
        digitalWrite(PWMR, LOW);
        digitalWrite(DIRL, LOW);
        digitalWrite(DIRR, LOW);
    }
};
