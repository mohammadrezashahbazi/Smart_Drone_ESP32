/*
 * Smart Drone ESP32 - Main Controller
 * Advanced ESP32 Drone with WiFi/GSM/SMS Control
 * GitHub: https://github.com/mohammadrezashahbazi/Smart_Drone_ESP32
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <IRremote.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

// Include custom headers
#include "gsm_module.h"
#include "sensors.h"
#include "safety_system.h"
#include "logger.h"
#include "remote_control.h"
#include "debug_tools.h"

// WiFi credentials
const char* ssid = "Smart_Drone_AP";
const char* password = "123456789";

// Web server
AsyncWebServer server(80);

// Flight modes
enum FlightMode {
  STANDBY,
  ARM,
  TAKEOFF,
  HOVER,
  MANUAL,
  AUTOPILOT,
  MISSION,
  RTH,
  LANDING,
  EMERGENCY_LAND
};

FlightMode currentMode = STANDBY;

// Global variables
bool isArmed = false;
bool isFlying = false;
float batteryVoltage = 0.0;
float altitude = 0.0;
double latitude = 0.0;
double longitude = 0.0;

// Mission waypoints
struct Waypoint {
  double lat;
  double lon;
  float alt;
  int delay_ms;
};

Waypoint mission[10];
int missionIndex = 0;
int missionLength = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize components
  initializeHardware();
  initializeWiFi();
  initializeWebServer();
  initializeGSM();
  initializeSensors();
  initializeSafetySystem();
  initializeLogger();
  initializeRemoteControl();
  
  Serial.println("Smart Drone ESP32 initialized successfully!");
  logEvent("SYSTEM_START", "Drone initialized");
}

void loop() {
  // Main flight control loop
  updateSensors();
  checkSafetyConditions();
  handleRemoteControl();
  handleGSMCommands();
  
  // Flight mode execution
  switch(currentMode) {
    case STANDBY:
      handleStandbyMode();
      break;
    case ARM:
      handleArmMode();
      break;
    case TAKEOFF:
      handleTakeoffMode();
      break;
    case HOVER:
      handleHoverMode();
      break;
    case MANUAL:
      handleManualMode();
      break;
    case AUTOPILOT:
      handleAutopilotMode();
      break;
    case MISSION:
      handleMissionMode();
      break;
    case RTH:
      handleRTHMode();
      break;
    case LANDING:
      handleLandingMode();
      break;
    case EMERGENCY_LAND:
      handleEmergencyLanding();
      break;
  }
  
  // Update telemetry
  updateTelemetry();
  
  delay(50); // 20Hz main loop
}

void initializeHardware() {
  // Motor pins
  pinMode(2, OUTPUT);  // Motor 1
  pinMode(4, OUTPUT);  // Motor 2
  pinMode(5, OUTPUT);  // Motor 3
  pinMode(18, OUTPUT); // Motor 4
  
  // LED indicators
  pinMode(19, OUTPUT); // Status LED
  pinMode(21, OUTPUT); // Armed LED
  
  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Initialize SD card
  if (!SD.begin()) {
    Serial.println("SD Card initialization failed!");
  }
}

void initializeWiFi() {
  WiFi.softAP(ssid, password);
  Serial.print("Access Point started: ");
  Serial.println(WiFi.softAPIP());
}

void initializeWebServer() {
  // Serve main control page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getMainPage());
  });
  
  // API endpoints
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getStatusJSON());
  });
  
  server.on("/api/arm", HTTP_POST, [](AsyncWebServerRequest *request){
    armDrone();
    request->send(200, "application/json", "{\"status\":\"armed\"}");
  });
  
  server.on("/api/takeoff", HTTP_POST, [](AsyncWebServerRequest *request){
    takeoff();
    request->send(200, "application/json", "{\"status\":\"takeoff\"}");
  });
  
  server.on("/api/land", HTTP_POST, [](AsyncWebServerRequest *request){
    land();
    request->send(200, "application/json", "{\"status\":\"landing\"}");
  });
  
  server.on("/api/emergency", HTTP_POST, [](AsyncWebServerRequest *request){
    emergencyLand();
    request->send(200, "application/json", "{\"status\":\"emergency\"}");
  });
  
  server.begin();
}

void armDrone() {
  if (currentMode == STANDBY && safetyCheck()) {
    isArmed = true;
    currentMode = ARM;
    digitalWrite(21, HIGH); // Armed LED
    logEvent("ARM", "Drone armed");
  }
}

void takeoff() {
  if (currentMode == ARM && isArmed) {
    currentMode = TAKEOFF;
    isFlying = true;
    logEvent("TAKEOFF", "Takeoff initiated");
  }
}

void land() {
  if (isFlying) {
    currentMode = LANDING;
    logEvent("LANDING", "Landing initiated");
  }
}

void emergencyLand() {
  currentMode = EMERGENCY_LAND;
  logEvent("EMERGENCY", "Emergency landing activated");
}

void handleStandbyMode() {
  // Standby mode - all motors off
  setMotorSpeed(0, 0, 0, 0);
  digitalWrite(19, LOW); // Status LED off
}

void handleArmMode() {
  // Armed mode - motors idle
  setMotorSpeed(100, 100, 100, 100); // Idle speed
  digitalWrite(19, HIGH); // Status LED on
}

void handleTakeoffMode() {
  // Takeoff sequence
  static unsigned long takeoffStart = 0;
  if (takeoffStart == 0) {
    takeoffStart = millis();
  }
  
  if (millis() - takeoffStart < 3000) {
    // Gradual takeoff
    int speed = map(millis() - takeoffStart, 0, 3000, 100, 200);
    setMotorSpeed(speed, speed, speed, speed);
  } else {
    currentMode = HOVER;
    takeoffStart = 0;
  }
}

void handleHoverMode() {
  // Hover mode - maintain altitude
  stabilizeAltitude(5.0); // 5 meters
}

void handleManualMode() {
  // Manual control via web interface or remote
  // Implementation depends on control input
}

void handleAutopilotMode() {
  // GPS-based autopilot
  // Navigate to target coordinates
}

void handleMissionMode() {
  // Execute waypoint mission
  if (missionIndex < missionLength) {
    navigateToWaypoint(mission[missionIndex]);
    if (reachedWaypoint(mission[missionIndex])) {
      missionIndex++;
    }
  } else {
    currentMode = RTH;
  }
}

void handleRTHMode() {
  // Return to home
  // Navigate back to takeoff point
}

void handleLandingMode() {
  // Landing sequence
  static unsigned long landingStart = 0;
  if (landingStart == 0) {
    landingStart = millis();
  }
  
  if (millis() - landingStart < 5000) {
    // Gradual landing
    int speed = map(millis() - landingStart, 0, 5000, 200, 0);
    setMotorSpeed(speed, speed, speed, speed);
  } else {
    currentMode = STANDBY;
    isArmed = false;
    isFlying = false;
    digitalWrite(21, LOW); // Armed LED off
    landingStart = 0;
  }
}

void handleEmergencyLanding() {
  // Emergency landing - fast descent
  setMotorSpeed(50, 50, 50, 50);
  delay(100);
  setMotorSpeed(0, 0, 0, 0);
  currentMode = STANDBY;
  isArmed = false;
  isFlying = false;
}

void setMotorSpeed(int m1, int m2, int m3, int m4) {
  analogWrite(2, m1);
  analogWrite(4, m2);
  analogWrite(5, m3);
  analogWrite(18, m4);
}

void stabilizeAltitude(float targetAltitude) {
  float error = targetAltitude - altitude;
  int correction = constrain(error * 10, -50, 50);
  setMotorSpeed(150 + correction, 150 + correction, 150 + correction, 150 + correction);
}

String getStatusJSON() {
  DynamicJsonDocument doc(1024);
  doc["mode"] = currentMode;
  doc["armed"] = isArmed;
  doc["flying"] = isFlying;
  doc["battery"] = batteryVoltage;
  doc["altitude"] = altitude;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["timestamp"] = millis();
  
  String output;
  serializeJson(doc, output);
  return output;
}

String getMainPage() {
  return R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Drone Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 0; padding: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; }
        .status { background: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
        .controls { background: white; padding: 20px; border-radius: 10px; }
        button { padding: 10px 20px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; }
        .btn-arm { background: #4CAF50; color: white; }
        .btn-takeoff { background: #2196F3; color: white; }
        .btn-land { background: #FF9800; color: white; }
        .btn-emergency { background: #f44336; color: white; }
        .gauge { display: inline-block; margin: 10px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 Smart Drone Control Panel</h1>
        
        <div class="status">
            <h2>Status</h2>
            <div id="status-content">Loading...</div>
        </div>
        
        <div class="controls">
            <h2>Controls</h2>
            <button class="btn-arm" onclick="sendCommand('arm')">ARM</button>
            <button class="btn-takeoff" onclick="sendCommand('takeoff')">TAKEOFF</button>
            <button class="btn-land" onclick="sendCommand('land')">LAND</button>
            <button class="btn-emergency" onclick="sendCommand('emergency')">EMERGENCY</button>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status-content').innerHTML = 
                        `Mode: ${data.mode}<br>
                         Armed: ${data.armed}<br>
                         Flying: ${data.flying}<br>
                         Battery: ${data.battery}V<br>
                         Altitude: ${data.altitude}m<br>
                         GPS: ${data.latitude}, ${data.longitude}`;
                });
        }
        
        function sendCommand(cmd) {
            fetch('/api/' + cmd, {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log(data));
        }
        
        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
)";
}
