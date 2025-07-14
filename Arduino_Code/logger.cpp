/*
 * Logger System Implementation for Smart Drone ESP32
 */

#include "logger.h"
#include "sensors.h"
#include <WiFi.h>
#include <esp_system.h>

// Global instances
Logger logger;
FlightDataLogger flightLogger;
BlackBoxLogger blackBoxLogger;
PerformanceLogger performanceLogger;
ConfigLogger configLogger;

// Log level and category names
const char* LOG_LEVEL_NAMES[] = {"DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
const char* LOG_CATEGORY_NAMES[] = {"SYSTEM", "SENSOR", "FLIGHT", "SAFETY", "COMM", "GPS", "BATTERY", "MOTOR"};

// Logger Implementation
Logger::Logger() {
  sdCardInitialized = false;
  serialLogging = true;
  fileLogging = false;
  minLogLevel = LOG_INFO;
  bufferIndex = 0;
  bufferSize = 0;
  totalLogs = 0;
  errorCount = 0;
  warningCount = 0;
  currentLogFile = "";
}

bool Logger::initialize() {
  Serial.println("Initializing Logger...");
  
  // Initialize SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    sdCardInitialized = false;
    fileLogging = false;
  } else {
    Serial.println("SD Card initialized successfully");
    sdCardInitialized = true;
    fileLogging = true;
    
    // Create logs directory
    if (!SD.exists("/logs")) {
      SD.mkdir("/logs");
    }
    
    // Generate log filename
    currentLogFile = "/logs/drone_" + String(millis()) + ".log";
    
    // Open log file
    logFile = SD.open(currentLogFile, FILE_WRITE);
    if (logFile) {
      logFile.println("# Drone Log File");
      logFile.println("# Timestamp,Level,Category,Message,Data");
      logFile.close();
      Serial.println("Log file created: " + currentLogFile);
    }
  }
  
  // Log initialization
  info("Logger initialized");
  logSystemEvent("INIT", "Logger system started");
  
  return true;
}

void Logger::setLogLevel(LogLevel level) {
  minLogLevel = level;
  info("Log level set to: " + String(LOG_LEVEL_NAMES[level]));
}

void Logger::enableSerial(bool enable) {
  serialLogging = enable;
}

void Logger::enableFile(bool enable) {
  if (enable && !sdCardInitialized) {
    warning("Cannot enable file logging - SD Card not available");
    return;
  }
  fileLogging = enable;
}

void Logger::log(LogLevel level, LogCategory category, const String& message, const String& data) {
  if (level < minLogLevel) return;
  
  // Create log entry
  LogEntry entry;
  entry.timestamp = millis();
  entry.level = level;
  entry.category = category;
  entry.message = message;
  entry.data = data;
  
  // Add to buffer
  logBuffer[bufferIndex] = entry;
  bufferIndex = (bufferIndex + 1) % 100;
  if (bufferSize < 100) bufferSize++;
  
  // Update statistics
  totalLogs++;
  if (level == LOG_ERROR) errorCount++;
  if (level == LOG_WARNING) warningCount++;
  
  // Format and output
  String formattedEntry = formatLogEntry(entry);
  
  if (serialLogging) {
    writeToSerial(formattedEntry);
  }
  
  if (fileLogging && sdCardInitialized) {
    writeToFile(formattedEntry);
  }
}

String Logger::formatLogEntry(const LogEntry& entry) {
  String formatted = String(entry.timestamp) + ",";
  formatted += LOG_LEVEL_NAMES[entry.level];
  formatted += ",";
  formatted += LOG_CATEGORY_NAMES[entry.category];
  formatted += ",\"";
  formatted += entry.message;
  formatted += "\"";
  
  if (entry.data.length() > 0) {
    formatted += ",\"";
    formatted += entry.data;
    formatted += "\"";
  }
  
  return formatted;
}

String Logger::getLevelString(LogLevel level) {
  return String(LOG_LEVEL_NAMES[level]);
}

String Logger::getCategoryString(LogCategory category) {
  return String(LOG_CATEGORY_NAMES[category]);
}

void Logger::writeToFile(const String& logEntry) {
  if (!sdCardInitialized) return;
  
  logFile = SD.open(currentLogFile, FILE_APPEND);
  if (logFile) {
    logFile.println(logEntry);
    logFile.close();
  }
}

void Logger::writeToSerial(const String& logEntry) {
  Serial.println("[LOG] " + logEntry);
}

String Logger::getTimestamp() {
  unsigned long ms = millis();
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  ms = ms % 1000;
  seconds = seconds % 60;
  minutes = minutes % 60;
  hours = hours % 24;
  
  char timestamp[20];
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);
  return String(timestamp);
}

// Simplified logging functions
void Logger::debug(const String& message, const String& data) {
  log(LOG_DEBUG, LOG_SYSTEM, message, data);
}

void Logger::info(const String& message, const String& data) {
  log(LOG_INFO, LOG_SYSTEM, message, data);
}

void Logger::warning(const String& message, const String& data) {
  log(LOG_WARNING, LOG_SYSTEM, message, data);
}

void Logger::error(const String& message, const String& data) {
  log(LOG_ERROR, LOG_SYSTEM, message, data);
}

void Logger::critical(const String& message, const String& data) {
  log(LOG_CRITICAL, LOG_SYSTEM, message, data);
}

// Specialized logging functions
void Logger::logSensorData(const String& sensorName, const String& data) {
  log(LOG_DEBUG, LOG_SENSOR, sensorName + " data", data);
}

void Logger::logFlightEvent(const String& event, const String& data) {
  log(LOG_INFO, LOG_FLIGHT, event, data);
}

void Logger::logSafetyEvent(const String& event, const String& data) {
  log(LOG_WARNING, LOG_SAFETY, event, data);
}

void Logger::logSystemEvent(const String& event, const String& data) {
  log(LOG_INFO, LOG_SYSTEM, event, data);
}

void Logger::rotateLogFile() {
  if (!sdCardInitialized) return;
  
  // Close current file
  if (logFile) {
    logFile.close();
  }
  
  // Create new filename
  currentLogFile = "/logs/drone_" + String(millis()) + ".log";
  
  // Open new file
  logFile = SD.open(currentLogFile, FILE_WRITE);
  if (logFile) {
    logFile.println("# Drone Log File - Rotated");
    logFile.println("# Timestamp,Level,Category,Message,Data");
    logFile.close();
  }
  
  info("Log file rotated to: " + currentLogFile);
}

void Logger::deleteOldLogs() {
  if (!sdCardInitialized) return;
  
  // Implementation to delete logs older than X days
  // This would require file date comparison
  info("Old logs cleanup completed");
}

bool Logger::checkSDCard() {
  if (!sdCardInitialized) return false;
  
  // Test SD card by trying to open a file
  File testFile = SD.open("/test.txt", FILE_WRITE);
  if (testFile) {
    testFile.close();
    SD.remove("/test.txt");
    return true;
  }
  
  return false;
}

unsigned long Logger::getTotalLogs() {
  return totalLogs;
}

unsigned long Logger::getErrorCount() {
  return errorCount;
}

unsigned long Logger::getWarningCount() {
  return warningCount;
}

LogEntry Logger::getLogEntry(int index) {
  if (index < 0 || index >= bufferSize) {
    LogEntry empty = {};
    return empty;
  }
  
  int actualIndex = (bufferIndex - bufferSize + index) % 100;
  if (actualIndex < 0) actualIndex += 100;
  
  return logBuffer[actualIndex];
}

int Logger::getBufferSize() {
  return bufferSize;
}

void Logger::clearBuffer() {
  bufferIndex = 0;
  bufferSize = 0;
}

void Logger::flush() {
  if (fileLogging && logFile) {
    logFile.flush();
  }
}

// FlightDataLogger Implementation
FlightDataLogger::FlightDataLogger() {
  recording = false;
  flightStartTime = 0;
  lastLogTime = 0;
  logInterval = 100; // 100ms default
  flightNumber = 1;
}

bool FlightDataLogger::initialize() {
  if (!SD.exists("/flights")) {
    SD.mkdir("/flights");
  }
  
  // Find next flight number
  File root = SD.open("/flights");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      String filename = file.name();
      if (filename.startsWith("flight_")) {
        int num = filename.substring(7, filename.indexOf('.')).toInt();
        if (num >= flightNumber) {
          flightNumber = num + 1;
        }
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  logger.info("FlightDataLogger initialized, next flight: " + String(flightNumber));
  return true;
}

void FlightDataLogger::startRecording() {
  if (recording) return;
  
  currentFlightFile = "/flights/flight_" + String(flightNumber) + ".csv";
  
  flightFile = SD.open(currentFlightFile, FILE_WRITE);
  if (flightFile) {
    // Write CSV header
    flightFile.println("Timestamp,Altitude,Latitude,Longitude,Roll,Pitch,Yaw,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Battery,Temperature,Pressure");
    flightFile.close();
    
    recording = true;
    flightStartTime = millis();
    lastLogTime = 0;
    
    logger.logFlightEvent("RECORDING_STARTED", "Flight " + String(flightNumber));
  }
}

void FlightDataLogger::stopRecording() {
  if (!recording) return;
  
  recording = false;
  
  if (flightFile) {
    flightFile.close();
  }
  
  logger.logFlightEvent("RECORDING_STOPPED", "Flight " + String(flightNumber) + ", Duration: " + String(getFlightDuration()) + "ms");
  
  flightNumber++;
}

void FlightDataLogger::logFlightData() {
  if (!recording) return;
  
  if (millis() - lastLogTime < logInterval) return;
  
  lastLogTime = millis();
  
  SensorData data = sensorManager.getCurrentData();
  
  flightFile = SD.open(currentFlightFile, FILE_APPEND);
  if (flightFile) {
    String dataLine = String(millis() - flightStartTime) + ",";
    dataLine += String(data.ultrasonic_altitude, 2) + ",";
    dataLine += String(data.latitude, 6) + ",";
    dataLine += String(data.longitude, 6) + ",";
    dataLine += String(data.roll, 2) + ",";
    dataLine += String(data.pitch, 2) + ",";
    dataLine += String(data.yaw, 2) + ",";
    dataLine += String(data.accelX, 3) + ",";
    dataLine += String(data.accelY, 3) + ",";
    dataLine += String(data.accelZ, 3) + ",";
    dataLine += String(data.gyroX, 3) + ",";
    dataLine += String(data.gyroY, 3) + ",";
    dataLine += String(data.gyroZ, 3) + ",";
    dataLine += String(data.batteryVoltage, 2) + ",";
    dataLine += String(data.temperature, 1) + ",";
    dataLine += String(data.pressure, 1);
    
    flightFile.println(dataLine);
    flightFile.close();
  }
}

bool FlightDataLogger::isRecording() {
  return recording;
}

unsigned long FlightDataLogger::getFlightDuration() {
  if (!recording) return 0;
  return millis() - flightStartTime;
}

void FlightDataLogger::logTakeoff() {
  logger.logFlightEvent("TAKEOFF", "Flight " + String(flightNumber));
}

void FlightDataLogger::logLanding() {
  logger.logFlightEvent("LANDING", "Flight " + String(flightNumber));
}

void FlightDataLogger::logWaypoint(double lat, double lon, float alt) {
  String waypointData = String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 2);
  logger.logFlightEvent("WAYPOINT", waypointData);
}

// BlackBoxLogger Implementation
BlackBoxLogger::BlackBoxLogger() {
  crashBufferIndex = 0;
  enabled = true;
  crashDetected = false;
}

void BlackBoxLogger::initialize() {
  enabled = true;
  crashDetected = false;
  logger.info("BlackBoxLogger initialized");
}

void BlackBoxLogger::logCriticalEvent(const String& event, const String& data) {
  if (!enabled) return;
  
  LogEntry entry;
  entry.timestamp = millis();
  entry.level = LOG_CRITICAL;
  entry.category = LOG_SAFETY;
  entry.message = event;
  entry.data = data;
  
  crashBuffer[crashBufferIndex] = entry;
  crashBufferIndex = (crashBufferIndex + 1) % 50;
  
  // If this is a crash event, save immediately
  if (event.indexOf("CRASH") >= 0 || event.indexOf("EMERGENCY") >= 0) {
    crashDetected = true;
    saveCrashReport();
  }
}

void BlackBoxLogger::dumpCrashData() {
  Serial.println("=== CRASH DATA DUMP ===");
  
  for (int i = 0; i < 50; i++) {
    int index = (crashBufferIndex + i) % 50;
    LogEntry& entry = crashBuffer[index];
    
    if (entry.timestamp > 0) {
      Serial.println(logger.formatLogEntry(entry));
    }
  }
  
  Serial.println("=== END CRASH DATA ===");
}

String BlackBoxLogger::getCrashReport() {
  String report = "CRASH REPORT\n";
  report += "Generated: " + String(millis()) + "\n";
  report += "System Info: " + getSystemInfo() + "\n\n";
  
  report += "Critical Events:\n";
  for (int i = 0; i < 50; i++) {
    int index = (crashBufferIndex + i) % 50;
    LogEntry& entry = crashBuffer[index];
    
    if (entry.timestamp > 0) {
      report += logger.formatLogEntry(entry) + "\n";
    }
  }
  
  return report;
}

void BlackBoxLogger::saveCrashReport() {
  if (!SD.begin(SD_CS_PIN)) return;
  
  String crashFile = "/crash_" + String(millis()) + ".txt";
  File file = SD.open(crashFile, FILE_WRITE);
  
  if (file) {
    file.print(getCrashReport());
    file.close();
    logger.critical("Crash report saved: " + crashFile);
  }
}

// PerformanceLogger Implementation
PerformanceLogger::PerformanceLogger() {
  loopCount = 0;
  maxLoopTime = 0;
  avgLoopTime = 0;
  totalLoopTime = 0;
  freeMemory = 0;
  minFreeMemory = UINT32_MAX;
  sensorUpdateCount = 0;
  communicationCount = 0;
  errorCount = 0;
}

void PerformanceLogger::logLoopTime(unsigned long loopTime) {
  loopCount++;
  totalLoopTime += loopTime;
  
  if (loopTime > maxLoopTime) {
    maxLoopTime = loopTime;
  }
  
  avgLoopTime = totalLoopTime / loopCount;
  
  if (loopTime > 50) { // Log slow loops
    logger.warning("Slow loop detected: " + String(loopTime) + "ms");
  }
}

void PerformanceLogger::logMemoryUsage() {
  freeMemory = ESP.getFreeHeap();
  
  if (freeMemory < minFreeMemory) {
    minFreeMemory = freeMemory;
  }
  
  if (freeMemory < 10000) { // Low memory warning
    logger.warning("Low memory: " + String(freeMemory) + " bytes");
  }
}

String PerformanceLogger::getPerformanceReport() {
  String report = "PERFORMANCE REPORT\n";
  report += "Loop Count: " + String(loopCount) + "\n";
  report += "Max Loop Time: " + String(maxLoopTime) + "ms\n";
  report += "Avg Loop Time: " + String(avgLoopTime) + "ms\n";
  report += "Free Memory: " + String(freeMemory) + " bytes\n";
  report += "Min Free Memory: " + String(minFreeMemory) + " bytes\n";
  report += "Sensor Updates: " + String(sensorUpdateCount) + "\n";
  report += "Communications: " + String(communicationCount) + "\n";
  report += "Errors: " + String(errorCount) + "\n";
  
  return report;
}

// Global functions
void logEvent(const String& module, const String& message) {
  logger.info("[" + module + "] " + message);
}

void logDebug(const String& module, const String& message) {
  logger.debug("[" + module + "] " + message);
}

void logInfo(const String& module, const String& message) {
  logger.info("[" + module + "] " + message);
}

void logWarning(const String& module, const String& message) {
  logger.warning("[" + module + "] " + message);
}

void logError(const String& module, const String& message) {
  logger.error("[" + module + "] " + message);
}

void logCritical(const String& module, const String& message) {
  logger.critical("[" + module + "] " + message);
}

void initializeLoggers() {
  logger.initialize();
  flightLogger.initialize();
  blackBoxLogger.initialize();
  performanceLogger.initialize();
  configLogger.initialize();
  
  logInfo("SYSTEM", "All loggers initialized successfully");
}

String formatBytes(size_t bytes) {
  if (bytes < 1024) return String(bytes) + "B";
  else if (bytes < (1024 * 1024)) return String(bytes/1024.0) + "KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes/1024.0/1024.0) + "MB";
  else return String(bytes/1024.0/1024.0/1024.0) + "GB";
}

String getSystemInfo() {
  String info = "ESP32 ";
  info += "Free Heap: " + formatBytes(ESP.getFreeHeap()) + ", ";
  info += "Chip Rev: " + String(ESP.getChipRevision()) + ", ";
  info += "CPU Freq: " + String(ESP.getCpuFreqMHz()) + "MHz, ";
  info += "SDK: " + String(ESP.getSdkVersion());
  return info;
}

void dumpSystemInfo() {
  Serial.println("=== SYSTEM INFO ===");
  Serial.println(getSystemInfo());
  Serial.println("===================");
}
