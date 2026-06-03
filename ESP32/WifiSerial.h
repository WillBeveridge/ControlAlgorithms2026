#pragma once
/*
 * WifiSerial.h
 * Mirrors Serial output over a WiFi TCP socket.
 * Drop this file into any sketch folder and include it.
 *
 * Usage:
 *   1. #include "WifiSerial.h"
 *   2. Declare an instance: WifiSerialClass WifiSerial;
 *   3. Call WifiSerial.begin() after WiFi is connected (in setup)
 *   4. Call WifiSerial.handle() in loop()
 *   5. Replace Serial.print / Serial.println with WifiSerial.print / WifiSerial.println
 *
 * Connect from Python:
 *   import socket
 *   s = socket.socket()
 *   s.connect(("ESP_IP_HERE", 8080))
 *   while True:
 *       data = s.recv(1024).decode()
 *       if data: print(data, end="")
 */

#include <WiFiServer.h>
#include <WiFiClient.h>

class WifiSerialClass {
public:
    // port: TCP port the mirror server listens on (default 8080)
    WifiSerialClass(int port = 8080) : _server(port) {}

    // Call once in setup(), after WiFi is connected
    void begin() {
        _server.begin();
    }

    // Call every iteration of loop() to accept and maintain the connection
    void handle() {
        if (!_client || !_client.connected()) {
            _client = _server.available();
        }
    }

    void print(String msg) {
        Serial.print(msg);
        send(msg);
    }

    void println(String msg) {
        Serial.println(msg);
        send(msg + "\n");
    }

    void println() {
        Serial.println();
        send("\n");
    }

    // Pass-through for int, float, etc. — mirrors Serial behaviour
    void print(int val)    { print(String(val));   }
    void print(float val)  { print(String(val));   }
    void print(char val)   { print(String(val));   }
    void println(int val)  { println(String(val)); }
    void println(float val){ println(String(val)); }
    void println(char val) { println(String(val)); }

    // Returns true if a Python client is currently connected
    bool clientConnected() {
        return _client && _client.connected();
    }

private:
    WiFiServer _server;
    WiFiClient _client;

    void send(String msg) {
        if (_client && _client.connected()) {
            _client.print(msg);
        }
    }
};
