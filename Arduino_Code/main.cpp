/*
 * Smart Drone ESP32 - Main Application
 * Advanced Flight Control System
 * Version: 2.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>

// Project headers
#include "sensors.h"
#include "motors.h"
#include "flight_control.h"
#include "safety_system.h"
#include "logger.h"
#include "communication.h"
#include "navigation.h"
#include "config.h"

// Global objects
SensorManager sensorManager;
MotorController motorController;
FlightController flightController;
SafetySystem safetySystem;
CommunicationManager commManager;
NavigationSystem navSystem;
WebServer server(80);

// System variables
unsigned long lastLoopTime = 0;
unsigned long loopCounter = 0;
unsigned long lastPerformanceCheck = 0;
unsigned long lastTelemetryUpdate = 0;
unsigned long lastSafetyCheck = 0;
unsigned long lastLogRotation = 0;

// Flight state
bool systemInitialized = false;
bool emergencyMode = false;
bool maintenanceMode = false;
FlightMode currentFlightMode = FLIGHT_MODE_MANUAL;

// Performance metrics
unsigned long averageLoopTime = 0;
unsigned long maxLoopTime = 0;
unsigned long totalLoopTime = 0;

// Function prototypes
void initializeSystem();
void systemLoop();
void handleEmergency();
void updateTelemetry();
void performSafetyChecks();
void handleWebRequests();
void performMaintenance();
void rotateLogsIfNeeded();
void printSystemStatus();
void handleOTA();
void setupWebServer();
void handleWebRoot();
void handleWebAPI();
void handleWebTelemetry();
void handleWebConfig();
void handleWebLogs();
void handleWebControl();
void handleWebCalibration();
void handleWebUpdate();
void handleWebRestart();
void handleWebFactoryReset();
void handleWebFileUpload();
void handleWebFileDelete();
void handleWebFileList();
void handleWebNotFound();
void checkSystemHealth();
void updatePerformanceMetrics();
void handleSerialCommands();
void processRemoteCommands();
void updateFlightParameters();
void manageFlightModes();
void handleFailsafe();
void performPreflightChecks();
void logSystemMetrics();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart Drone ESP32 Starting ===");
  
  // Initialize system
  initializeSystem();
  
  // Setup web server
  setupWebServer();
  
  // Initialize OTA
  handleOTA();
  
  Serial.println("=== System Ready ===");
  logger.info("System initialization completed");
  
  // Start main loop timer
  lastLoopTime = millis();
}

void loop() {
  unsigned long loopStart = millis();
  
  // Handle emergency conditions first
  if (emergencyMode) {
    handleEmergency();
    return;
  }
  
  // Main system loop
  systemLoop();
  
  // Handle web server
  server.handleClient();
  
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Handle serial commands
  handleSerialCommands();
  
  // Update performance metrics
  updatePerformanceMetrics();
  
  // Calculate loop time
  unsigned long loopTime = millis() - loopStart;
  performanceLogger.logLoopTime(loopTime);
  
  // Update counters
  loopCounter++;
  totalLoopTime += loopTime;
  
  if (loopTime > maxLoopTime) {
    maxLoopTime = loopTime;
  }
  
  // Check for slow loops
  if (loopTime > MAIN_LOOP_WARNING_TIME) {
    logger.warning("Slow main loop: " + String(loopTime) + "ms");
  }
  
  // Maintain target loop frequency
  while (millis() - loopStart < MAIN_LOOP_INTERVAL) {
    delayMicroseconds(100);
  }
}

void initializeSystem() {
  Serial.println("Initializing system components...");
  
  // Initialize loggers first
  initializeLoggers();
  
  // Initialize sensors
  if (!sensorManager.initialize()) {
    logger.error("Sensor initialization failed");
    emergencyMode = true;
    return;
  }
  
  // Initialize motors
  if (!motorController.initialize()) {
    logger.error("Motor initialization failed");
    emergencyMode = true;
    return;
  }
  
  // Initialize flight controller
  if (!flightController.initialize()) {
    logger.error("Flight controller initialization failed");
    emergencyMode = true;
    return;
  }
  
  // Initialize safety system
  if (!safetySystem.initialize()) {
    logger.error("Safety system initialization failed");
    emergencyMode = true;
    return;
  }
  
  // Initialize communication
  if (!commManager.initialize()) {
    logger.warning("Communication initialization failed - continuing without telemetry");
  }
  
  // Initialize navigation
  if (!navSystem.initialize()) {
    logger.warning("Navigation initialization failed - GPS features disabled");
  }
  
  // Configure WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  
  // Try to connect to saved WiFi
  if (strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      logger.info("WiFi connected: " + WiFi.localIP().toString());
    } else {
      logger.warning("WiFi connection failed - AP mode only");
    }
  }
  
  // Initialize mDNS
  if (MDNS.begin("smartdrone")) {
    logger.info("mDNS responder started");
  }
  
  // Perform pre-flight checks
  performPreflightChecks();
  
  systemInitialized = true;
  logger.info("System initialization completed successfully");
}

void systemLoop() {
  unsigned long currentTime = millis();
  
  // Update sensors
  sensorManager.update();
  
  // Update safety system
  if (currentTime - lastSafetyCheck >= SAFETY_CHECK_INTERVAL) {
    performSafetyChecks();
    lastSafetyCheck = currentTime;
  }
  
  // Update flight controller
  flightController.update();
  
  // Update navigation
  navSystem.update();
  
  // Update motors
  motorController.update();
  
  // Update telemetry
  if (currentTime - lastTelemetryUpdate >= TELEMETRY_UPDATE_INTERVAL) {
    updateTelemetry();
    lastTelemetryUpdate = currentTime;
  }
  
  // Process remote commands
  processRemoteCommands();
  
  // Update flight parameters
  updateFlightParameters();
  
  // Manage flight modes
  manageFlightModes();
  
  // Performance check
  if (currentTime - lastPerformanceCheck >= PERFORMANCE_CHECK_INTERVAL) {
    checkSystemHealth();
    lastPerformanceCheck = currentTime;
  }
  
  // Log rotation
  if (currentTime - lastLogRotation >= LOG_ROTATION_INTERVAL) {
    rotateLogsIfNeeded();
    lastLogRotation = currentTime;
  }
  
  // Log system metrics periodically
  if (loopCounter % 1000 == 0) {
    logSystemMetrics();
  }
}

void handleEmergency() {
  static unsigned long lastEmergencyLog = 0;
  
  // Emergency landing sequence
  motorController.emergencyStop();
  
  // Log emergency state
  if (millis() - lastEmergencyLog > 1000) {
    logger.critical("Emergency mode active");
    blackBoxLogger.logCriticalEvent("EMERGENCY_MODE", "System in emergency state");
    lastEmergencyLog = millis();
  }
  
  // Flash emergency LED
  static bool ledState = false;
  static unsigned long lastLedToggle = 0;
  if (millis() - lastLedToggle > 200) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    lastLedToggle = millis();
  }
  
  // Check for emergency recovery
  if (safetySystem.canRecoverFromEmergency()) {
    logger.info("Emergency recovery initiated");
    emergencyMode = false;
    safetySystem.resetEmergencyState();
  }
}

void updateTelemetry() {
  if (!commManager.isConnected()) return;
  
  // Create telemetry data
  JsonDocument telemetryDoc;
  SensorData data = sensorManager.getCurrentData();
  MotorStatus motorStatus = motorController.getStatus();
  FlightStatus flightStatus = flightController.getStatus();
  SafetyStatus safetyStatus = safetySystem.getStatus();
  
  // System info
  telemetryDoc["timestamp"] = millis();
  telemetryDoc["flight_mode"] = currentFlightMode;
  telemetryDoc["armed"] = flightController.isArmed();
  telemetryDoc["emergency"] = emergencyMode;
  
  // Sensor data
  JsonObject sensors = telemetryDoc["sensors"];
  sensors["altitude"] = data.ultrasonic_altitude;
  sensors["temperature"] = data.temperature;
  sensors["pressure"] = data.pressure;
  sensors["humidity"] = data.humidity;
  sensors["battery"] = data.batteryVoltage;
  sensors["gps_lat"] = data.latitude;
  sensors["gps_lon"] = data.longitude;
  sensors["gps_alt"] = data.gps_altitude;
  sensors["gps_fix"] = data.gps_fix;
  sensors["satellites"] = data.satellites;
  
  // IMU data
  JsonObject imu = telemetryDoc["imu"];
  imu["roll"] = data.roll;
  imu["pitch"] = data.pitch;
  imu["yaw"] = data.yaw;
  imu["accel_x"] = data.accelX;
  imu["accel_y"] = data.accelY;
  imu["accel_z"] = data.accelZ;
  imu["gyro_x"] = data.gyroX;
  imu["gyro_y"] = data.gyroY;
  imu["gyro_z"] = data.gyroZ;
  
  // Motor status
  JsonObject motors = telemetryDoc["motors"];
  motors["motor1"] = motorStatus.motor1Speed;
  motors["motor2"] = motorStatus.motor2Speed;
  motors["motor3"] = motorStatus.motor3Speed;
  motors["motor4"] = motorStatus.motor4Speed;
  motors["armed"] = motorStatus.armed;
  
  // Flight status
  JsonObject flight = telemetryDoc["flight"];
  flight["throttle"] = flightStatus.throttle;
  flight["roll"] = flightStatus.roll;
  flight["pitch"] = flightStatus.pitch;
  flight["yaw"] = flightStatus.yaw;
  flight["altitude_hold"] = flightStatus.altitudeHold;
  flight["heading_hold"] = flightStatus.headingHold;
  
  // Safety status
  JsonObject safety = telemetryDoc["safety"];
  safety["state"] = safetyStatus.state;
  safety["battery_low"] = safetyStatus.batteryLow;
  safety["altitude_warning"] = safetyStatus.altitudeWarning;
  safety["gps_warning"] = safetyStatus.gpsWarning;
  safety["geofence_warning"] = safetyStatus.geofenceWarning;
  
  // Performance metrics
  JsonObject performance = telemetryDoc["performance"];
  performance["loop_time"] = averageLoopTime;
  performance["max_loop_time"] = maxLoopTime;
  performance["free_heap"] = ESP.getFreeHeap();
  performance["uptime"] = millis();
  
  // Send telemetry
  String telemetryJson;
  serializeJson(telemetryDoc, telemetryJson);
  commManager.sendTelemetry(telemetryJson);
}

void performSafetyChecks() {
  safetySystem.update();
  
  SafetyStatus status = safetySystem.getStatus();
  
  // Check for emergency conditions
  if (status.state == SAFETY_EMERGENCY_LAND) {
    logger.critical("Emergency landing triggered");
    flightController.emergencyLand();
    emergencyMode = true;
  }
  
  // Check for warnings
  if (status.batteryLow) {
    logger.warning("Battery low warning");
  }
  
  if (status.altitudeWarning) {
    logger.warning("Altitude warning");
  }
  
  if (status.gpsWarning) {
    logger.warning("GPS warning");
  }
  
  if (status.geofenceWarning) {
    logger.warning("Geofence warning");
  }
}

void setupWebServer() {
  // Web interface routes
  server.on("/", handleWebRoot);
  server.on("/api", HTTP_GET, handleWebAPI);
  server.on("/telemetry", HTTP_GET, handleWebTelemetry);
  server.on("/config", HTTP_GET, handleWebConfig);
  server.on("/config", HTTP_POST, handleWebConfig);
  server.on("/logs", HTTP_GET, handleWebLogs);
  server.on("/control", HTTP_POST, handleWebControl);
  server.on("/calibrate", HTTP_POST, handleWebCalibration);
  server.on("/update", HTTP_POST, handleWebUpdate);
  server.on("/restart", HTTP_POST, handleWebRestart);
  server.on("/factory-reset", HTTP_POST, handleWebFactoryReset);
  server.on("/upload", HTTP_POST, handleWebFileUpload);
  server.on("/delete", HTTP_POST, handleWebFileDelete);
  server.on("/files", HTTP_GET, handleWebFileList);
  server.onNotFound(handleWebNotFound);
  
  server.begin();
  logger.info("Web server started on port 80");
}

void handleWebRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Drone Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; }
        .container { max-width: 800px; margin: 0 auto; }
        .status { background: #f0f0f0; padding: 10px; border-radius: 5px; margin: 10px 0; }
        .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 20px 0; }
        button { padding: 10px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }
        .btn-primary { background: #007bff; color: white; }
        .btn-danger { background: #dc3545; color: white; }
        .btn-warning { background: #ffc107; color: black; }
        .btn-success { background: #28a745; color: white; }
        #telemetry { background: #f8f9fa; padding: 15px; border-radius: 5px; font-family: monospace; }
        .nav { background: #343a40; padding: 10px; border-radius: 5px; margin-bottom: 20px; }
        .nav a { color: white; text-decoration: none; margin-right: 20px; }
        .nav a:hover { text-decoration: underline; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Smart Drone Control Panel</h1>
        
        <div class="nav">
            <a href="/">Home</a>
            <a href="/config">Configuration</a>
            <a href="/logs">Logs</a>
            <a href="/files">Files</a>
        </div>
        
        <div class="status">
            <h3>System Status</h3>
            <div id="status">Loading...</div>
        </div>
        
        <div class="controls">
            <button class="btn-primary" onclick="arm()">ARM</button>
            <button class="btn-danger" onclick="disarm()">DISARM</button>
            <button class="btn-warning" onclick="takeoff()">TAKEOFF</button>
            <button class="btn-warning" onclick="land()">LAND</button>
            <button class="btn-success" onclick="calibrate()">CALIBRATE</button>
            <button class="btn-danger" onclick="emergency()">EMERGENCY</button>
        </div>
        
        <div id="telemetry">
            <h3>Live Telemetry</h3>
            <div id="telemetry-data">Connecting...</div>
        </div>
    </div>
    
    <script>
        function updateStatus() {
            fetch('/api').then(r => r.json()).then(data => {
                document.getElementById('status').innerHTML = 
                    'Flight Mode: ' + data.flight_mode + '<br>' +
                    'Armed: ' + data.armed + '<br>' +
                    'Emergency: ' + data.emergency + '<br>' +
                    'Battery: ' + data.battery.toFixed(1) + 'V<br>' +
                    'Altitude: ' + data.altitude.toFixed(1) + 'm<br>' +
                    'GPS: ' + data.gps_fix + ' (' + data.satellites + ' sats)';
            });
        }
        
        function updateTelemetry() {
            fetch('/telemetry').then(r => r.json()).then(data => {
                document.getElementById('telemetry-data').innerHTML = 
                    JSON.stringify(data, null, 2);
            });
        }
        
        function arm() { fetch('/control', {method: 'POST', body: 'action=arm'}); }
        function disarm() { fetch('/control', {method: 'POST', body: 'action=disarm'}); }
        function takeoff() { fetch('/control', {method: 'POST', body: 'action=takeoff'}); }
        function land() { fetch('/control', {method: 'POST', body: 'action=land'}); }
        function calibrate() { fetch('/calibrate', {method: 'POST'}); }
        function emergency() { fetch('/control', {method: 'POST', body: 'action=emergency'}); }
        
        setInterval(updateStatus, 1000);
        setInterval(updateTelemetry, 2000);
        updateStatus();
        updateTelemetry();
    </script>
</body>
</html>
)";
  
  server.send(200, "text/html", html);
}

void handleWebAPI() {
  JsonDocument doc;
  SensorData data = sensorManager.getCurrentData();
  FlightStatus flightStatus = flightController.getStatus();
  
  doc["flight_mode"] = currentFlightMode;
  doc["armed"] = flightController.isArmed();
  doc["emergency"] = emergencyMode;
  doc["battery"] = data.batteryVoltage;
  doc["altitude"] = data.ultrasonic_altitude;
  doc["gps_fix"] = data.gps_fix;
  doc["satellites"] = data.satellites;
  doc["uptime"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWebTelemetry() {
  JsonDocument doc;
  SensorData data = sensorManager.getCurrentData();
  MotorStatus motorStatus = motorController.getStatus();
  FlightStatus flightStatus = flightController.getStatus();
  SafetyStatus safetyStatus = safetySystem.getStatus();
  
  // Fill telemetry data (same as updateTelemetry function)
  doc["timestamp"] = millis();
  doc["flight_mode"] = currentFlightMode;
  doc["armed"] = flightController.isArmed();
  doc["emergency"] = emergencyMode;
  
  // Add sensor data
  JsonObject sensors = doc["sensors"];
  sensors["altitude"] = data.ultrasonic_altitude;
  sensors["temperature"] = data.temperature;
  sensors["pressure"] = data.pressure;
  sensors["battery"] = data.batteryVoltage;
  sensors["gps_lat"] = data.latitude;
  sensors["gps_lon"] = data.longitude;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWebControl() {
  String action = server.arg("action");
  
  if (action == "arm") {
    flightController.arm();
    server.send(200, "text/plain", "Armed");
    logger.info("Web command: ARM");
  } else if (action == "disarm") {
    flightController.disarm();
    server.send(200, "text/plain", "Disarmed");
    logger.info("Web command: DISARM");
  } else if (action == "takeoff") {
    flightController.takeoff();
    server.send(200, "text/plain", "Takeoff initiated");
    logger.info("Web command: TAKEOFF");
  } else if (action == "land") {
    flightController.land();
    server.send(200, "text/plain", "Landing initiated");
    logger.info("Web command: LAND");
  } else if (action == "emergency") {
    emergencyMode = true;
    server.send(200, "text/plain", "Emergency mode activated");
    logger.critical("Web command: EMERGENCY");
  } else {
    server.send(400, "text/plain", "Unknown action");
  }
}

void handleWebCalibration() {
  logger.info("Web command: CALIBRATE");
  
  // Start calibration
  sensorManager.startCalibration();
  
  server.send(200, "text/plain", "Calibration started");
}

void handleWebNotFound() {
  server.send(404, "text/plain", "File not found");
}

void handleOTA() {
  ArduinoOTA.setHostname("smartdrone");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  
  ArduinoOTA.onStart([]() {
    logger.info("OTA Update started");
  });
  
  ArduinoOTA.onEnd([]() {
    logger.info("OTA Update completed");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned long lastProgress = 0;
    if (millis() - lastProgress > 1000) {
      logger.info("OTA Progress: " + String(progress * 100 / total) + "%");
      lastProgress = millis();
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    logger.error("OTA Error: " + String(error));
  });
  
  ArduinoOTA.begin();
}

void checkSystemHealth() {
  // Check memory usage
  size_t freeHeap = ESP.getFreeHeap();
  performanceLogger.logMemoryUsage();
  
  if (freeHeap < CRITICAL_MEMORY_THRESHOLD) {
    logger.critical("Critical memory shortage: " + String(freeHeap) + " bytes");
    emergencyMode = true;
  } else if (freeHeap < LOW_MEMORY_THRESHOLD) {
    logger.warning("Low memory warning: " + String(freeHeap) + " bytes");
  }
  
  // Check loop performance
  averageLoopTime = totalLoopTime / loopCounter;
  if (averageLoopTime > SLOW_LOOP_THRESHOLD) {
    logger.warning("System performance degraded - avg loop: " + String(averageLoopTime) + "ms");
  }
  
  // Check sensor health
  if (!sensorManager.isHealthy()) {
    logger.error("Sensor system unhealthy");
    safetySystem.triggerSensorFailure();
  }
}

void updatePerformanceMetrics() {
  performanceLogger.logMemoryUsage();
  
  // Reset max loop time periodically
  if (loopCounter % 10000 == 0) {
    maxLoopTime = 0;
    totalLoopTime = 0;
    loopCounter = 0;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      printSystemStatus();
    } else if (command == "arm") {
      flightController.arm();
      Serial.println("Armed");
    } else if (command == "disarm") {
      flightController.disarm();
      Serial.println("Disarmed");
    } else if (command == "emergency") {
      emergencyMode = true;
      Serial.println("Emergency mode activated");
    } else if (command == "restart") {
      ESP.restart();
    } else if (command == "calibrate") {
      sensorManager.startCalibration();
      Serial.println("Calibration started");
    } else if (command == "logs") {
      for (int i = 0; i < logger.getBufferSize(); i++) {
        LogEntry entry = logger.getLogEntry(i);
        Serial.println(String(entry.timestamp) + " " + entry.message);
      }
    } else if (command == "performance") {
      Serial.println(performanceLogger.getPerformanceReport());
    } else if (command == "help") {
      Serial.println("Commands: status, arm, disarm, emergency, restart, calibrate, logs, performance, help");
    } else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}

void printSystemStatus() {
  SensorData data = sensorManager.getCurrentData();
  FlightStatus flightStatus = flightController.getStatus();
  MotorStatus motorStatus = motorController.getStatus();
  
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.println("Uptime: " + String(millis() / 1000) + " seconds");
  Serial.println("Flight Mode: " + String(currentFlightMode));
  Serial.println("Armed: " + String(flightController.isArmed() ? "YES" : "NO"));
  Serial.println("Emergency: " + String(emergencyMode ? "YES" : "NO"));
  Serial.println("Battery: " + String(data.batteryVoltage, 1) + "V");
  Serial.println("Altitude: " + String(data.ultrasonic_altitude, 1) + "m");
  Serial.println("GPS Fix: " + String(data.gps_fix ? "YES" : "NO"));
  Serial.println("Satellites: " + String(data.satellites));
  Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("Avg Loop Time: " + String(averageLoopTime) + "ms");
  Serial.println("Max Loop Time: " + String(maxLoopTime) + "ms");
  Serial.println("====================\n");
}

void processRemoteCommands() {
  if (commManager.hasNewCommand()) {
    String command = commManager.getCommand();
    
    // Process command
    if (command == "ARM") {
      flightController.arm();
      logger.info("Remote command: ARM");
    } else if (command == "DISARM") {
      flightController.disarm();
      logger.info("Remote command: DISARM");
    } else if (command == "TAKEOFF") {
      flightController.takeoff();
      logger.info("Remote command: TAKEOFF");
    } else if (command == "LAND") {
      flightController.land();
      logger.info("Remote command: LAND");
    } else if (command == "RTH") {
      flightController.returnToHome();
      logger.info("Remote command: RTH");
    } else if (command == "EMERGENCY") {
      emergencyMode = true;
      logger.critical("Remote command: EMERGENCY");
    }
  }
}

void updateFlightParameters() {
  // Update flight controller with latest sensor data
  SensorData data = sensorManager.getCurrentData();
  flightController.updateSensorData(data);
  
  // Update navigation system
  navSystem.updatePosition(data.latitude, data.longitude, data.gps_altitude);
  navSystem.updateOrientation(data.roll, data.pitch, data.yaw);
}

void manageFlightModes() {
  // Flight mode management logic
  FlightMode requestedMode = flightController.getRequestedMode();
  
  if (requestedMode != currentFlightMode) {
    // Check if mode change is safe
    if (safetySystem.canChangeModeFrom(currentFlightMode, requestedMode)) {
      currentFlightMode = requestedMode;
      logger.info("Flight mode changed to: " + String(currentFlightMode));
    } else {
      logger.warning("Flight mode change denied by safety system");
    }
  }
}

void handleFailsafe() {
  if (safetySystem.isFailsafeActive()) {
    logger.warning("Failsafe activated");
    flightController.activateFailsafe();
  }
}

void performPreflightChecks() {
  logger.info("Performing pre-flight checks...");
  
  bool allChecksPass = true;
  
  // Check sensors
  if (!sensorManager.isHealthy()) {
    logger.error("Pre-flight: Sensor check failed");
    allChecksPass = false;
  }
  
  // Check motors
  if (!motorController.isHealthy()) {
    logger.error("Pre-flight: Motor check failed");
    allChecksPass = false;
  }
  
  // Check battery
  SensorData data = sensorManager.getCurrentData();
  if (data.batteryVoltage < BATTERY_MIN_VOLTAGE) {
    logger.error("Pre-flight: Battery voltage too low");
    allChecksPass = false;
  }
  
  // Check GPS
  if (!data.gps_fix) {
    logger.warning("Pre-flight: GPS fix not available");
  }
  
  // Check calibration
  if (!sensorManager.isCalibrated()) {
    logger.warning("Pre-flight: Sensors not calibrated");
  }
  
  if (allChecksPass) {
    logger.info("Pre-flight checks passed");
  } else {
    logger.error("Pre-flight checks failed");
    emergencyMode = true;
  }
}

void rotateLogsIfNeeded() {
  // Check if log rotation is needed
  if (logger.getCurrentLogFile().length() > 0) {
    // Check file size or time criteria
    logger.rotateLogFile();
  }
}

void logSystemMetrics() {
  JsonDocument metricsDoc;
  
  // System metrics
  metricsDoc["uptime"] = millis();
  metricsDoc["free_heap"] = ESP.getFreeHeap();
  metricsDoc["loop_count"] = loopCounter;
  metricsDoc["avg_loop_time"] = averageLoopTime;
  metricsDoc["max_loop_time"] = maxLoopTime;
  
  // Sensor metrics
  SensorData data = sensorManager.getCurrentData();
  metricsDoc["battery"] = data.batteryVoltage;
  metricsDoc["temperature"] = data.temperature;
  metricsDoc["altitude"] = data.ultrasonic_altitude;
  metricsDoc["gps_fix"] = data.gps_fix;
  
  // Flight metrics
  FlightStatus flightStatus = flightController.getStatus();
  metricsDoc["flight_mode"] = currentFlightMode;
  metricsDoc["armed"] = flightController.isArmed();
  metricsDoc["throttle"] = flightStatus.throttle;
  
  // Motor metrics
  MotorStatus motorStatus = motorController.getStatus();
  metricsDoc["motor1"] = motorStatus.motor1Speed;
  metricsDoc["motor2"] = motorStatus.motor2Speed;
  metricsDoc["motor3"] = motorStatus.motor3Speed;
  metricsDoc["motor4"] = motorStatus.motor4Speed;
  
  // Safety metrics
  SafetyStatus safetyStatus = safetySystem.getStatus();
  metricsDoc["safety_state"] = safetyStatus.state;
  metricsDoc["emergency"] = emergencyMode;
  
  // Log metrics
  String metricsJson;
  serializeJson(metricsDoc, metricsJson);
  performanceLogger.logMetrics(metricsJson);
}

// Additional web handlers implementation
void handleWebConfig() {
  if (server.method() == HTTP_GET) {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Drone Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; }
        .container { max-width: 600px; margin: 0 auto; }
        .form-group { margin: 15px 0; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input, select { width: 100%; padding: 8px; border: 1px solid #ccc; border-radius: 4px; }
        button { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .nav { background: #343a40; padding: 10px; border-radius: 5px; margin-bottom: 20px; }
        .nav a { color: white; text-decoration: none; margin-right: 20px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="nav">
            <a href="/">Home</a>
            <a href="/config">Configuration</a>
            <a href="/logs">Logs</a>
            <a href="/files">Files</a>
        </div>
        
        <h1>Drone Configuration</h1>
        
        <form method="post">
            <div class="form-group">
                <label>WiFi SSID:</label>
                <input type="text" name="wifi_ssid" value="">
            </div>
            
            <div class="form-group">
                <label>WiFi Password:</label>
                <input type="password" name="wifi_password">
            </div>
            
            <div class="form-group">
                <label>Flight Mode:</label>
                <select name="flight_mode">
                    <option value="0">Manual</option>
                    <option value="1">Stabilize</option>
                    <option value="2">Altitude Hold</option>
                    <option value="3">Position Hold</option>
                    <option value="4">Auto</option>
                </select>
            </div>
            
            <div class="form-group">
                <label>Max Altitude (m):</label>
                <input type="number" name="max_altitude" value="50" min="1" max="200">
            </div>
            
            <div class="form-group">
                <label>Max Distance (m):</label>
                <input type="number" name="max_distance" value="100" min="1" max="1000">
            </div>
            
            <div class="form-group">
                <label>Battery Warning Voltage:</label>
                <input type="number" step="0.1" name="battery_warning" value="10.5" min="9.0" max="12.0">
            </div>
            
            <div class="form-group">
                <label>PID Gains - Roll P:</label>
                <input type="number" step="0.01" name="pid_roll_p" value="1.0" min="0" max="10">
            </div>
            
            <div class="form-group">
                <label>PID Gains - Roll I:</label>
                <input type="number" step="0.01" name="pid_roll_i" value="0.1" min="0" max="1">
            </div>
            
            <div class="form-group">
                <label>PID Gains - Roll D:</label>
                <input type="number" step="0.01" name="pid_roll_d" value="0.05" min="0" max="1">
            </div>
            
            <button type="submit">Save Configuration</button>
        </form>
    </div>
</body>
</html>
)";
    server.send(200, "text/html", html);
  } else if (server.method() == HTTP_POST) {
    // Handle configuration update
    String wifiSSID = server.arg("wifi_ssid");
    String wifiPassword = server.arg("wifi_password");
    int flightMode = server.arg("flight_mode").toInt();
    float maxAltitude = server.arg("max_altitude").toFloat();
    float maxDistance = server.arg("max_distance").toFloat();
    float batteryWarning = server.arg("battery_warning").toFloat();
    float pidRollP = server.arg("pid_roll_p").toFloat();
    float pidRollI = server.arg("pid_roll_i").toFloat();
    float pidRollD = server.arg("pid_roll_d").toFloat();
    
    // Save configuration
    saveConfiguration(wifiSSID, wifiPassword, flightMode, maxAltitude, maxDistance, 
                     batteryWarning, pidRollP, pidRollI, pidRollD);
    
    server.send(200, "text/plain", "Configuration saved. Restart required.");
    logger.info("Configuration updated via web interface");
  }
}

void handleWebLogs() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>System Logs</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; }
        .container { max-width: 1000px; margin: 0 auto; }
        .nav { background: #343a40; padding: 10px; border-radius: 5px; margin-bottom: 20px; }
        .nav a { color: white; text-decoration: none; margin-right: 20px; }
        .log-container { background: #f8f9fa; padding: 15px; border-radius: 5px; max-height: 600px; overflow-y: auto; }
        .log-entry { margin: 5px 0; padding: 5px; border-left: 3px solid #ccc; }
        .log-info { border-left-color: #17a2b8; }
        .log-warning { border-left-color: #ffc107; background: #fff3cd; }
        .log-error { border-left-color: #dc3545; background: #f8d7da; }
        .log-critical { border-left-color: #dc3545; background: #f5c6cb; font-weight: bold; }
        .controls { margin: 10px 0; }
        button { padding: 8px 15px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #007bff; color: white; }
        .btn-danger { background: #dc3545; color: white; }
    </style>
</head>
<body>
    <div class="container">
        <div class="nav">
            <a href="/">Home</a>
            <a href="/config">Configuration</a>
            <a href="/logs">Logs</a>
            <a href="/files">Files</a>
        </div>
        
        <h1>System Logs</h1>
        
        <div class="controls">
            <button class="btn-primary" onclick="refreshLogs()">Refresh</button>
            <button class="btn-danger" onclick="clearLogs()">Clear Logs</button>
            <label>
                Auto-refresh: <input type="checkbox" id="autoRefresh" onchange="toggleAutoRefresh()">
            </label>
        </div>
        
        <div class="log-container" id="logContainer">
            Loading logs...
        </div>
    </div>
    
    <script>
        let autoRefreshInterval;
        
        function refreshLogs() {
            fetch('/api/logs').then(r => r.json()).then(data => {
                let html = '';
                data.logs.forEach(log => {
                    let className = 'log-entry log-' + log.level.toLowerCase();
                    html += '<div class="' + className + '">' +
                           '[' + new Date(log.timestamp).toLocaleString() + '] ' +
                           log.level + ': ' + log.message + '</div>';
                });
                document.getElementById('logContainer').innerHTML = html;
                document.getElementById('logContainer').scrollTop = 
                    document.getElementById('logContainer').scrollHeight;
            });
        }
        
        function clearLogs() {
            if (confirm('Are you sure you want to clear all logs?')) {
                fetch('/api/logs', {method: 'DELETE'}).then(() => {
                    refreshLogs();
                });
            }
        }
        
        function toggleAutoRefresh() {
            const checkbox = document.getElementById('autoRefresh');
            if (checkbox.checked) {
                autoRefreshInterval = setInterval(refreshLogs, 2000);
            } else {
                clearInterval(autoRefreshInterval);
            }
        }
        
        refreshLogs();
    </script>
</body>
</html>
)";
  
  server.send(200, "text/html", html);
}

void handleWebFileList() {
  JsonDocument doc;
  JsonArray files = doc["files"];
  
  // List SD card files if available
  if (SD.begin()) {
    File root = SD.open("/");
    File file = root.openNextFile();
    while (file) {
      JsonObject fileObj = files.add<JsonObject>();
      fileObj["name"] = file.name();
      fileObj["size"] = file.size();
      fileObj["isDirectory"] = file.isDirectory();
      file = root.openNextFile();
    }
    root.close();
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWebFileUpload() {
  HTTPUpload& upload = server.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    
    if (SD.begin()) {
      File file = SD.open(filename, FILE_WRITE);
      if (file) {
        logger.info("File upload started: " + filename);
      } else {
        logger.error("Failed to create file: " + filename);
        server.send(500, "text/plain", "Failed to create file");
        return;
      }
      file.close();
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (SD.begin()) {
      File file = SD.open("/" + upload.filename, FILE_APPEND);
      if (file) {
        file.write(upload.buf, upload.currentSize);
        file.close();
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    logger.info("File upload completed: " + upload.filename + " (" + String(upload.totalSize) + " bytes)");
    server.send(200, "text/plain", "File uploaded successfully");
  }
}

void handleWebFileDelete() {
  String filename = server.arg("filename");
  
  if (SD.begin() && SD.remove(filename)) {
    server.send(200, "text/plain", "File deleted successfully");
    logger.info("File deleted: " + filename);
  } else {
    server.send(500, "text/plain", "Failed to delete file");
    logger.error("Failed to delete file: " + filename);
  }
}

void handleWebUpdate() {
  if (server.hasArg("firmware")) {
    // Handle firmware update
    String firmwareData = server.arg("firmware");
    
    if (Update.begin(firmwareData.length())) {
      size_t written = Update.write((uint8_t*)firmwareData.c_str(), firmwareData.length());
      
      if (written == firmwareData.length()) {
        if (Update.end(true)) {
          server.send(200, "text/plain", "Update successful. Rebooting...");
          logger.info("Firmware update completed successfully");
          delay(1000);
          ESP.restart();
        } else {
          server.send(500, "text/plain", "Update failed: " + Update.errorString());
          logger.error("Firmware update failed: " + Update.errorString());
        }
      } else {
        server.send(500, "text/plain", "Update failed: Write error");
        logger.error("Firmware update failed: Write error");
      }
    } else {
      server.send(500, "text/plain", "Update failed: Not enough space");
      logger.error("Firmware update failed: Not enough space");
    }
  } else {
    server.send(400, "text/plain", "No firmware data provided");
  }
}

void handleWebRestart() {
  server.send(200, "text/plain", "Restarting system...");
  logger.info("System restart requested via web interface");
  delay(1000);
  ESP.restart();
}

void handleWebFactoryReset() {
  server.send(200, "text/plain", "Factory reset initiated...");
  logger.info("Factory reset requested via web interface");
  
  // Clear all saved configuration
  clearConfiguration();
  
  // Clear logs
  logger.clearAllLogs();
  
  // Reset to defaults
  resetToFactoryDefaults();
  
  delay(2000);
  ESP.restart();
}

// Configuration management functions
void saveConfiguration(String wifiSSID, String wifiPassword, int flightMode, 
                      float maxAltitude, float maxDistance, float batteryWarning,
                      float pidRollP, float pidRollI, float pidRollD) {
  JsonDocument configDoc;
  
  configDoc["wifi_ssid"] = wifiSSID;
  configDoc["wifi_password"] = wifiPassword;
  configDoc["flight_mode"] = flightMode;
  configDoc["max_altitude"] = maxAltitude;
  configDoc["max_distance"] = maxDistance;
  configDoc["battery_warning"] = batteryWarning;
  configDoc["pid_roll_p"] = pidRollP;
  configDoc["pid_roll_i"] = pidRollI;
  configDoc["pid_roll_d"] = pidRollD;
  
  // Save to EEPROM or SD card
  if (SD.begin()) {
    File configFile = SD.open("/config.json", FILE_WRITE);
    if (configFile) {
      serializeJson(configDoc, configFile);
      configFile.close();
      logger.info("Configuration saved to SD card");
    }
  }
  
  // Apply configuration
  flightController.updatePIDGains(pidRollP, pidRollI, pidRollD);
  safetySystem.updateLimits(maxAltitude, maxDistance, batteryWarning);
}

void loadConfiguration() {
  if (SD.begin()) {
    File configFile = SD.open("/config.json");
    if (configFile) {
      JsonDocument configDoc;
      DeserializationError error = deserializeJson(configDoc, configFile);
      configFile.close();
      
      if (!error) {
        // Apply loaded configuration
        if (configDoc.containsKey("flight_mode")) {
          currentFlightMode = (FlightMode)configDoc["flight_mode"].as<int>();
        }
        
        if (configDoc.containsKey("pid_roll_p")) {
          float pidP = configDoc["pid_roll_p"];
          float pidI = configDoc["pid_roll_i"];
          float pidD = configDoc["pid_roll_d"];
          flightController.updatePIDGains(pidP, pidI, pidD);
        }
        
        if (configDoc.containsKey("max_altitude")) {
          float maxAlt = configDoc["max_altitude"];
          float maxDist = configDoc["max_distance"];
          float battWarn = configDoc["battery_warning"];
          safetySystem.updateLimits(maxAlt, maxDist, battWarn);
        }
        
        logger.info("Configuration loaded successfully");
      } else {
        logger.error("Failed to parse configuration file");
      }
    } else {
      logger.info("No configuration file found, using defaults");
    }
  }
}

void clearConfiguration() {
  if (SD.begin()) {
    SD.remove("/config.json");
    logger.info("Configuration file cleared");
  }
}

void resetToFactoryDefaults() {
  // Reset all systems to default values
  currentFlightMode = FLIGHT_MODE_MANUAL;
  emergencyMode = false;
  maintenanceMode = false;
  
  // Reset flight controller
  flightController.resetToDefaults();
  
  // Reset safety system
  safetySystem.resetToDefaults();
  
  // Reset sensor calibration
  sensorManager.resetCalibration();
  
  logger.info("System reset to factory defaults");
}

// Performance monitoring functions
void monitorSystemPerformance() {
  static unsigned long lastMemoryCheck = 0;
  static unsigned long lastCPUCheck = 0;
  
  unsigned long currentTime = millis();
  
  // Memory monitoring
  if (currentTime - lastMemoryCheck > 5000) {
    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    float memoryUsage = ((float)(totalHeap - freeHeap) / totalHeap) * 100;
    
    performanceLogger.logMemoryUsage(freeHeap, memoryUsage);
    
    if (memoryUsage > 90) {
      logger.critical("Critical memory usage: " + String(memoryUsage, 1) + "%");
    } else if (memoryUsage > 80) {
      logger.warning("High memory usage: " + String(memoryUsage, 1) + "%");
    }
    
    lastMemoryCheck = currentTime;
  }
  
  // CPU monitoring
  if (currentTime - lastCPUCheck > 10000) {
    float cpuUsage = calculateCPUUsage();
    performanceLogger.logCPUUsage(cpuUsage);
    
    if (cpuUsage > 90) {
      logger.critical("Critical CPU usage: " + String(cpuUsage, 1) + "%");
    } else if (cpuUsage > 80) {
      logger.warning("High CPU usage: " + String(cpuUsage, 1) + "%");
    }
    
    lastCPUCheck = currentTime;
  }
}

float calculateCPUUsage() {
  static unsigned long lastIdleTime = 0;
  static unsigned long lastTotalTime = 0;
  
  unsigned long currentTime = millis();
  unsigned long idleTime = currentTime - (totalLoopTime / loopCounter);
  unsigned long totalTime = currentTime;
  
  float cpuUsage = 0;
  if (totalTime > lastTotalTime) {
    cpuUsage = ((float)(totalTime - lastTotalTime - (idleTime - lastIdleTime)) / 
                (totalTime - lastTotalTime)) * 100;
  }
  
  lastIdleTime = idleTime;
  lastTotalTime = totalTime;
  
  return cpuUsage;
}

// Watchdog functions
void initializeWatchdog() {
  // Enable watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  logger.info("Watchdog timer initialized");
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

// Main loop optimization
void optimizeMainLoop() {
  // Dynamic frequency adjustment based on system load
  if (averageLoopTime > TARGET_LOOP_TIME * 1.5) {
    // System overloaded - reduce non-critical operations
    TELEMETRY_UPDATE_INTERVAL *= 2;
    PERFORMANCE_CHECK_INTERVAL *= 2;
    logger.warning("System overloaded - reducing update frequencies");
  } else if (averageLoopTime < TARGET_LOOP_TIME * 0.5) {
    // System underloaded - increase update frequencies
    TELEMETRY_UPDATE_INTERVAL = max(100, TELEMETRY_UPDATE_INTERVAL / 2);
    PERFORMANCE_CHECK_INTERVAL = max(1000, PERFORMANCE_CHECK_INTERVAL / 2);
  }
}

// Error recovery functions
void handleSystemError(String errorMessage) {
  logger.error("System error: " + errorMessage);
  blackBoxLogger.logCriticalEvent("SYSTEM_ERROR", errorMessage);
  
  // Attempt recovery
  if (attemptSystemRecovery()) {
    logger.info("System recovery successful");
  } else {
    logger.critical("System recovery failed - entering emergency mode");
    emergencyMode = true;
  }
}

bool attemptSystemRecovery() {
  // Try to recover from various error conditions
  
  // Reset sensors
  if (!sensorManager.isHealthy()) {
    sensorManager.reset();
    delay(1000);
    if (!sensorManager.initialize()) {
      return false;
    }
  }
  
  // Reset communication
  if (!commManager.isConnected()) {
    commManager.reset();
    delay(1000);
    if (!commManager.initialize()) {
      logger.warning("Communication recovery failed");
    }
  }
  
  // Reset motors if safe
  if (flightController.isLanded()) {
    motorController.reset();
    delay(1000);
    if (!motorController.initialize()) {
      return false;
    }
  }
  
  return true;
}

// System shutdown functions
void shutdownSystem() {
  logger.info("System shutdown initiated");
  
  // Disarm motors
  flightController.disarm();
  motorController.stop();
  
  // Stop all sensors
  sensorManager.stop();
  
  // Close logs
  logger.flush();
  performanceLogger.flush();
  blackBoxLogger.flush();
  
  // Stop communication
  commManager.stop();
  
  // Stop web server
  server.stop();
  
  logger.info("System shutdown completed");
}

// Emergency procedures
void executeEmergencyProcedures() {
  static unsigned long emergencyStartTime = 0;
  
  if (emergencyStartTime == 0) {
    emergencyStartTime = millis();
    logger.critical("Emergency procedures activated");
    blackBoxLogger.logCriticalEvent("EMERGENCY_START", "Emergency procedures initiated");
  }
  
  // Emergency landing sequence
  flightController.emergencyLand();
  
  // Stop all non-essential systems
  commManager.setEmergencyMode(true);
  
  // Flash emergency beacon
  static bool beaconState = false;
  static unsigned long lastBeaconToggle = 0;
  if (millis() - lastBeaconToggle > 500) {
    beaconState = !beaconState;
    digitalWrite(EMERGENCY_LED_PIN, beaconState ? HIGH : LOW);
    lastBeaconToggle = millis();
  }
  
  // Send emergency signal
  if (commManager.isConnected()) {
    commManager.sendEmergencySignal();
  }
  
  // Log emergency telemetry
  if (millis() - emergencyStartTime > 60000) { // After 1 minute
    blackBoxLogger.logCriticalEvent("EMERGENCY_TIMEOUT", "Emergency mode active for over 1 minute");
    emergencyStartTime = millis(); // Reset timer
  }
}
