#include "communication.h"

// متغیرهای سراسری
CommunicationManager commManager;
RemoteControlData currentRCData;
TelemetryData currentTelemetry;

// سازنده
CommunicationManager::CommunicationManager() :
  server(WEB_SERVER_PORT),
  rcSerial(RC_RX_PIN, RC_TX_PIN),
  wifiConnected(false),
  lastTelemetryTime(0),
  lastRCTime(0),
  lastHeartbeat(0)
{
  ssid = AP_SSID;
  password = AP_PASSWORD;
  
  // مقداردهی اولیه داده‌ها
  memset(&telemetryData, 0, sizeof(telemetryData));
  memset(&rcData, 0, sizeof(rcData));
}

// راه‌اندازی
bool CommunicationManager::initialize() {
  Serial.println("Initializing Communication Manager...");
  
  // راه‌اندازی Serial برای RC
  rcSerial.begin(RC_BAUD_RATE);
  
  // راه‌اندازی WiFi
  setupWiFi();
  
  // راه‌اندازی WebServer
  setupWebServer();
  
  Serial.println("Communication Manager initialized successfully");
  return true;
}

// راه‌اندازی WiFi
void CommunicationManager::setupWiFi() {
  Serial.print("Setting up WiFi Access Point: ");
  Serial.println(ssid);
  
  WiFi.softAP(ssid.c_str(), password.c_str());
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  wifiConnected = true;
}

// راه‌اندازی WebServer
void CommunicationManager::setupWebServer() {
  // Handle CORS
  server.onNotFound([this]() {
    handleCORS();
  });
  
  // صفحه اصلی
  server.on("/", HTTP_GET, [this]() {
    handleRoot();
  });
  
  // API برای telemetry
  server.on("/api/telemetry", HTTP_GET, [this]() {
    handleTelemetry();
  });
  
  // API برای کنترل
  server.on("/api/control", HTTP_POST, [this]() {
    handleControl();
  });
  
  // شروع سرور
  server.begin();
  Serial.println("Web server started on port 80");
}

// صفحه اصلی
void CommunicationManager::handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Drone Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; }
        .status { background: white; padding: 20px; margin: 10px 0; border-radius: 10px; }
        .controls { background: white; padding: 20px; margin: 10px 0; border-radius: 10px; }
        button { padding: 10px 20px; margin: 5px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }
        .arm-btn { background: #4CAF50; color: white; }
        .disarm-btn { background: #f44336; color: white; }
        .emergency-btn { background: #FF9800; color: white; }
        .slider { width: 100%; }
        .value { font-weight: bold; color: #333; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 Drone Control Panel</h1>
        
        <div class="status">
            <h3>Status</h3>
            <p>Battery: <span id="battery" class="value">--</span>V</p>
            <p>Altitude: <span id="altitude" class="value">--</span>m</p>
            <p>Mode: <span id="mode" class="value">--</span></p>
            <p>Armed: <span id="armed" class="value">--</span></p>
        </div>
        
        <div class="controls">
            <h3>Controls</h3>
            <p>
                <button class="arm-btn" onclick="armDrone()">ARM</button>
                <button class="disarm-btn" onclick="disarmDrone()">DISARM</button>
                <button class="emergency-btn" onclick="emergency()">EMERGENCY</button>
            </p>
            
            <p>
                Throttle: <input type="range" id="throttle" class="slider" min="0" max="100" value="0">
                <span id="throttle-val">0</span>%
            </p>
            
            <p>
                Roll: <input type="range" id="roll" class="slider" min="-100" max="100" value="0">
                <span id="roll-val">0</span>
            </p>
            
            <p>
                Pitch: <input type="range" id="pitch" class="slider" min="-100" max="100" value="0">
                <span id="pitch-val">0</span>
            </p>
            
            <p>
                Yaw: <input type="range" id="yaw" class="slider" min="-100" max="100" value="0">
                <span id="yaw-val">0</span>
            </p>
        </div>
    </div>
    
    <script>
        // بروزرسانی مقادیر
        document.getElementById('throttle').oninput = function() {
            document.getElementById('throttle-val').innerHTML = this.value;
        }
        document.getElementById('roll').oninput = function() {
            document.getElementById('roll-val').innerHTML = this.value;
        }
        document.getElementById('pitch').oninput = function() {
            document.getElementById('pitch-val').innerHTML = this.value;
        }
        document.getElementById('yaw').oninput = function() {
            document.getElementById('yaw-val').innerHTML = this.value;
        }
        
        // توابع کنترل
        function armDrone() {
            fetch('/api/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'arm'})
            });
        }
        
        function disarmDrone() {
            fetch('/api/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'disarm'})
            });
        }
        
        function emergency() {
            fetch('/api/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'emergency'})
            });
        }
        
        // دریافت وضعیت
        function updateStatus() {
            fetch('/api/telemetry')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('battery').innerHTML = data.battery;
                    document.getElementById('altitude').innerHTML = data.altitude;
                    document.getElementById('mode').innerHTML = data.mode;
                    document.getElementById('armed').innerHTML = data.armed ? 'YES' : 'NO';
                });
        }
        
        // بروزرسانی هر 500ms
        setInterval(updateStatus, 500);
        updateStatus();
    </script>
</body>
</html>
)";
  
  server.send(200, "text/html", html);
}

// API Telemetry
void CommunicationManager::handleTelemetry() {
  handleCORS();
  
  String json = createTelemetryJSON();
  server.send(200, "application/json", json);
}

// API Control
void CommunicationManager::handleControl() {
  handleCORS();
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, body);
    
    String action = doc["action"];
    
    if (action == "arm") {
      rcData.armSwitch = true;
      server.send(200, "application/json", "{\"status\":\"armed\"}");
    } else if (action == "disarm") {
      rcData.armSwitch = false;
      server.send(200, "application/json", "{\"status\":\"disarmed\"}");
    } else if (action == "emergency") {
      rcData.killSwitch = true;
      server.send(200, "application/json", "{\"status\":\"emergency\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"unknown action\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"no data\"}");
  }
}

// Handle CORS
void CommunicationManager::handleCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  
  if (server.method() == HTTP_OPTIONS) {
    server.send(200);
  }
}

// به‌روزرسانی
void CommunicationManager::update() {
  unsigned long currentTime = millis();
  
  // بروزرسانی WebServer
  server.handleClient();
  
  // دریافت داده‌های RC
  if (rcSerial.available()) {
    String rcInput = rcSerial.readStringUntil('\n');
    if (parseRCData(rcInput)) {
      lastRCTime = currentTime;
      rcData.isValid = true;
    }
  }
  
  // بررسی timeout RC
  if (currentTime - lastRCTime > RC_TIMEOUT) {
    rcData.isValid = false;
  }
  
  // ارسال heartbeat
  if (currentTime - lastHeartbeat > HEARTBEAT_INTERVAL) {
    lastHeartbeat = currentTime;
    // ارسال heartbeat به RC
    rcSerial.println("HEARTBEAT");
  }
}

// تجزیه داده‌های RC
bool CommunicationManager::parseRCData(String data) {
  // فرمت: "RC,roll,pitch,yaw,throttle,arm,mode,kill"
  if (!data.startsWith("RC,")) return false;
  
  int indices[8];
  int count = 0;
  
  for (int i = 0; i < data.length() && count < 8; i++) {
    if (data[i] == ',' || i == data.length() - 1) {
      indices[count++] = i;
    }
  }
  
  if (count < 7) return false;
  
  try {
    rcData.roll = data.substring(indices[0] + 1, indices[1]).toFloat();
    rcData.pitch = data.substring(indices[1] + 1, indices[2]).toFloat();
    rcData.yaw = data.substring(indices[2] + 1, indices[3]).toFloat();
    rcData.throttle = data.substring(indices[3] + 1, indices[4]).toFloat();
    rcData.armSwitch = data.substring(indices[4] + 1, indices[5]).toInt();
    rcData.modeSwitch = data.substring(indices[5] + 1, indices[6]).toInt();
    rcData.killSwitch = data.substring(indices[6] + 1, indices[7]).toInt();
    
    rcData.timestamp = millis();
    return true;
  } catch (...) {
    return false;
  }
}

// ایجاد JSON telemetry
String CommunicationManager::createTelemetryJSON() {
  DynamicJsonDocument doc(1024);
  
  doc["roll"] = telemetryData.roll;
  doc["pitch"] = telemetryData.pitch;
  doc["yaw"] = telemetryData.yaw;
  doc["altitude"] = telemetryData.altitude;
  doc["latitude"] = telemetryData.latitude;
  doc["longitude"] = telemetryData.longitude;
  doc["battery"] = telemetryData.batteryVoltage;
  doc["mode"] = telemetryData.flightMode;
  doc["armed"] = telemetryData.isArmed;
  doc["timestamp"] = telemetryData.timestamp;
  
  String output;
  serializeJson(doc, output);
  return output;
}

// ارسال telemetry
void CommunicationManager::sendTelemetry(const TelemetryData& data) {
  telemetryData = data;
  lastTelemetryTime = millis();
  
  // ارسال به RC
  String telemetryString = "TEL,";
  telemetryString += String(data.roll) + ",";
  telemetryString += String(data.pitch) + ",";
  telemetryString += String(data.yaw) + ",";
  telemetryString += String(data.altitude) + ",";
  telemetryString += String(data.batteryVoltage) + ",";
  telemetryString += String(data.flightMode) + ",";
  telemetryString += String(data.isArmed);
  
  rcSerial.println(telemetryString);
}

// دریافت داده‌های RC
RemoteControlData CommunicationManager::getRemoteControlData() {
  return rcData;
}

// وضعیت اتصال RC
bool CommunicationManager::isRCConnected() {
  return rcData.isValid && (millis() - lastRCTime < RC_TIMEOUT);
}

// وضعیت WiFi
bool CommunicationManager::isWiFiConnected() {
  return wifiConnected;
}

// چاپ وضعیت
void CommunicationManager::printStatus() {
  Serial.println("=== Communication Status ===");
  Serial.print("WiFi: ");
  Serial.println(wifiConnected ? "Connected" : "Disconnected");
  Serial.print("RC: ");
  Serial.println(isRCConnected() ? "Connected" : "Disconnected");
  Serial.print("Last RC: ");
  Serial.println(millis() - lastRCTime);
  Serial.println("============================");
}

// توابع کمکی سراسری
void initializeCommunication() {
  commManager.initialize();
}

void updateCommunication() {
  commManager.update();
}

bool isRemoteControlActive() {
  return commManager.isRCConnected();
}

RemoteControlData getRemoteControlInput() {
  return commManager.getRemoteControlData();
}

void sendDroneTelemetry(const TelemetryData& data) {
  commManager.sendTelemetry(data);
}
