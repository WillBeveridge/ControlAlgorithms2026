#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char *ssid = "RobotWifi";
const char *password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Call this anywhere in your code to print to the browser
void serialPrint(String message) {
    ws.textAll(message);
}

const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Monitor</title>
    <style>
        body { background: #1e1e1e; color: #00ff00; font-family: monospace; padding: 10px; }
        #output { height: 85vh; overflow-y: auto; white-space: pre-wrap; border: 1px solid #444; padding: 10px; }
        #input-row { display: flex; gap: 8px; margin-top: 8px; }
        input { flex: 1; background: #2d2d2d; color: #00ff00; border: 1px solid #444; padding: 6px; font-family: monospace; }
        button { background: #444; color: #00ff00; border: 1px solid #666; padding: 6px 12px; cursor: pointer; }
    </style>
</head>
<body>
    <div id="output"></div>
    <div id="input-row">
        <input type="text" id="msg" placeholder="Send message..." onkeydown="if(event.key==='Enter') send()"/>
        <button onclick="send()">Send</button>
        <button onclick="document.getElementById('output').innerHTML=''">Clear</button>
    </div>
    <script>
        const out = document.getElementById('output');
        const ws = new WebSocket('ws://' + location.host + '/ws');
        
        ws.onmessage = (e) => {
            out.innerHTML += e.data + '\n';
            out.scrollTop = out.scrollHeight;
        };
        ws.onopen = () => out.innerHTML += '--- Connected ---\n';
        ws.onclose = () => out.innerHTML += '--- Disconnected ---\n';

        function send() {
            const input = document.getElementById('msg');
            if (input.value) {
                ws.send(input.value);
                input.value = '';
            }
        }
    </script>
</body>
</html>
)rawliteral";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        // message received from browser - handle it however you want
        String msg = String((char*)data).substring(0, len);
        serialPrint("Browser: " + msg);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    
    Serial.println("Connected! IP: " + WiFi.localIP().toString());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", HTML);
    });
    server.begin();
}

void loop() {
    // Example: mirror hardware Serial to the browser
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        serialPrint(msg);
    }

    // Call serialPrint() anywhere to send to browser
    // e.g. serialPrint("Robot position: " + String(x));
}