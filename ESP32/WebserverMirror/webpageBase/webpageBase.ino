#include <WiFi.h>           // Use <ESP8266WiFi.h> for ESP8266
#include <WebServer.h>      // Use <ESP8266WebServer.h> for ESP8266
#include <WebSocketsServer.h>

const char* ssid     = "RobotWifi";
const char* password = "12345678";

WebServer        server(80);
WebSocketsServer ws(81);

// ── Embedded HTML page ───────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP Monitor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      background: #1a1a1a;
      color: #f0f0f0;
      font-family: 'Courier New', monospace;
      display: flex;
      flex-direction: column;
      height: 100vh;
    }

    header {
      background: #111;
      padding: 10px 16px;
      display: flex;
      align-items: center;
      gap: 10px;
      border-bottom: 1px solid #333;
    }

    header h1 { font-size: 14px; color: #aaa; letter-spacing: 1px; }

    #status {
      margin-left: auto;
      font-size: 12px;
      display: flex;
      align-items: center;
      gap: 6px;
    }

    #dot {
      width: 8px; height: 8px;
      border-radius: 50%;
      background: #555;
      transition: background 0.3s;
    }

    #dot.connected  { background: #4caf50; }
    #dot.connecting { background: #ff9800; }

    #console {
      flex: 1;
      overflow-y: auto;
      padding: 12px 16px;
      display: flex;
      flex-direction: column;
      gap: 2px;
    }

    .line {
      display: flex;
      gap: 12px;
      font-size: 13px;
      line-height: 1.6;
      white-space: pre-wrap;
      word-break: break-all;
      animation: fadeIn 0.1s ease;
    }

    @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }

    .ts  { color: #555; flex-shrink: 0; user-select: none; }
    .msg { color: #e0e0e0; }
    .msg.warn  { color: #ff9800; }
    .msg.error { color: #f44336; }
    .msg.info  { color: #29b6f6; }
    .msg.ok    { color: #4caf50; }

    footer {
      background: #111;
      border-top: 1px solid #333;
      padding: 8px 16px;
      display: flex;
      gap: 8px;
    }

    footer input {
      flex: 1;
      background: #222;
      border: 1px solid #444;
      border-radius: 4px;
      color: #f0f0f0;
      font-family: inherit;
      font-size: 13px;
      padding: 6px 10px;
      outline: none;
    }

    footer input:focus { border-color: #666; }

    footer button {
      background: #333;
      border: 1px solid #555;
      border-radius: 4px;
      color: #ccc;
      cursor: pointer;
      font-size: 12px;
      padding: 6px 12px;
    }

    footer button:hover { background: #444; }
  </style>
</head>
<body>

<header>
  <h1>&#9654; ESP SERIAL MONITOR</h1>
  <div id="status">
    <div id="dot" class="connecting"></div>
    <span id="status-text">Connecting…</span>
  </div>
</header>

<div id="console"></div>

<footer>
  <input id="input" type="text" placeholder="Send a message to ESP…" />
  <button onclick="sendMsg()">Send</button>
  <button onclick="clearConsole()">Clear</button>
</footer>

<script>
  const consoleEl = document.getElementById('console');
  const dot        = document.getElementById('dot');
  const statusText = document.getElementById('status-text');
  const input      = document.getElementById('input');

  let ws;

  function timestamp() {
    const d = new Date();
    const pad = n => String(n).padStart(2, '0');
    return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${String(d.getMilliseconds()).padStart(3,'0')}`;
  }

  function colorClass(text) {
    const t = text.toLowerCase();
    if (t.includes('error') || t.includes('fail')) return 'error';
    if (t.includes('warn'))                          return 'warn';
    if (t.includes('connected') || t.includes('ok')) return 'ok';
    if (t.includes('ip') || t.includes('http'))      return 'info';
    return '';
  }

  function addLine(text, cls = '') {
    const line = document.createElement('div');
    line.className = 'line';

    const ts  = document.createElement('span');
    ts.className = 'ts';
    ts.textContent = timestamp();

    const msg = document.createElement('span');
    msg.className = 'msg ' + (cls || colorClass(text));
    msg.textContent = text;

    line.appendChild(ts);
    line.appendChild(msg);
    consoleEl.appendChild(line);
    consoleEl.scrollTop = consoleEl.scrollHeight;
  }

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');

    ws.onopen = () => {
      dot.className = 'connected';
      statusText.textContent = 'Connected';
      addLine('WebSocket connected.', 'ok');
    };

    ws.onmessage = e => addLine(e.data);

    ws.onclose = () => {
      dot.className = '';
      statusText.textContent = 'Disconnected — retrying…';
      addLine('Connection lost. Reconnecting in 2s…', 'warn');
      setTimeout(connect, 2000);
    };

    ws.onerror = () => ws.close();
  }

  function sendMsg() {
    const text = input.value.trim();
    if (!text || ws.readyState !== WebSocket.OPEN) return;
    ws.send(text);
    addLine('> ' + text, 'info');
    input.value = '';
  }

  function clearConsole() { consoleEl.innerHTML = ''; }

  input.addEventListener('keydown', e => { if (e.key === 'Enter') sendMsg(); });

  connect();
</script>
</body>
</html>
)rawliteral";

// ── Helpers ──────────────────────────────────────────────────────────────────

// Call this instead of Serial.println() to send to both Serial and the browser
void log(String msg) {
  Serial.println(msg);
  ws.broadcastTXT(msg);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    Serial.println("Browser: " + msg);
    // Echo back or handle the message however you like
    ws.broadcastTXT("Echo: " + msg);
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Open in browser: http://");
  Serial.println(WiFi.localIP());

  // Serve the HTML page
  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();

  // Start WebSocket server on port 81
  ws.begin();
  ws.onEvent(onWebSocketEvent);

  log("ESP ready. IP: " + WiFi.localIP().toString());
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  ws.loop();

  // Example: send a heartbeat every 5 seconds
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    log("Uptime: " + String(millis() / 1000) + "s");
    lastHeartbeat = millis();
  }
}
