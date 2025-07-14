/*
 * Logger System for Smart Drone ESP32
 * Comprehensive logging and data recording
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include <ArduinoJson.h>

// SD Card pins
#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

// Log levels
enum LogLevel {
  LOG_DEBUG = 0,
  LOG_INFO = 1,
  LOG_WARNING = 2,
  LOG_ERROR = 3,
  LOG_CRITICAL = 4
};

// Log categories
enum LogCategory {
  LOG_SYSTEM = 0,
  LOG_SENSOR = 1,
  LOG_FLIGHT = 2,
  LOG_SAFETY = 3,
  LOG_COMMUNICATION = 4,
  LOG_GPS = 5,
  LOG_BATTERY = 6,
  LOG_MOTOR = 7
};

// Log entry structure
struct LogEntry {
  unsigned long timestamp;
  LogLevel level;
  LogCategory category;
  String message;
  String data;
};

// Logger class
class Logger {
private:
  bool sdCardInitialized;
  bool serialLogging;
  bool fileLogging;
  
  String currentLogFile;
  File logFile;
  
  LogLevel minLogLevel;
  
  // Circular buffer for recent logs
  LogEntry logBuffer[100];
  int bufferIndex;
  int bufferSize;
  
  // Statistics
  unsigned long totalLogs;
  unsigned long errorCount;
  unsigned long warningCount;
  
  // Private methods
  String formatLogEntry(const LogEntry& entry);
  String getLevelString(LogLevel level);
  String getCategoryString(LogCategory category);
  void writeToFile(const String& logEntry);
  void writeToSerial(const String& logEntry);
  String getTimestamp();
  
public:
  Logger();
  bool initialize();
  void setLogLevel(LogLevel level);
  void enableSerial(bool enable);
  void enableFile(bool enable);
  
  // Main logging functions
  void log(LogLevel level, LogCategory category, const String& message, const String& data = "");
  void debug(const String& message, const String& data = "");
  void info(const String& message, const String& data = "");
  void warning(const String& message, const String& data = "");
  void error(const String& message, const String& data = "");
  void critical(const String& message, const String& data = "");
  
  // Specialized logging
  void logSensorData(const String& sensorName, const String& data);
  void logFlightEvent(const String& event, const String& data);
  void logSafetyEvent(const String& event, const String& data);
  void logSystemEvent(const String& event, const String& data);
  
  // File management
  void rotateLogFile();
  void deleteOldLogs();
  String getCurrentLogFile();
  bool checkSDCard();
  
  // Statistics
  unsigned long getTotalLogs();
  unsigned long getErrorCount();
  unsigned long getWarningCount();
  
  // Buffer access
  LogEntry getLogEntry(int index);
  int getBufferSize();
  void clearBuffer();
  
  // Export functions
  String exportLogsAsJSON();
  String exportLogsAsCSV();
  bool exportToFile(const String& filename, const String& format);
  
  // Flush functions
  void flush();
  void syncToSD();
};

// Flight data logger
class FlightDataLogger {
private:
  bool recording;
  String currentFlightFile;
  File flightFile;
  unsigned long flightStartTime;
  unsigned long lastLogTime;
  int logInterval;
  int flightNumber;
  
public:
  FlightDataLogger();
  bool initialize();
  
  void startRecording();
  void stopRecording();
  void logFlightData();
  
  bool isRecording();
  String getCurrentFlightFile();
  unsigned long getFlightDuration();
  int getFlightNumber();
  
  void setLogInterval(int interval);
  int getLogInterval();
  
  void logTakeoff();
  void logLanding();
  void logWaypoint(double lat, double lon, float alt);
};

// Black box logger for crash analysis
class BlackBoxLogger {
private:
  LogEntry crashBuffer[50];
  int crashBufferIndex;
  bool enabled;
  bool crashDetected;
  
public:
  BlackBoxLogger();
  void initialize();
  
  void enable(bool enable);
  void logCriticalEvent(const String& event, const String& data);
  void detectCrash();
  void dumpCrashData();
  void clearCrashData();
  
  bool hasCrashData();
  bool isCrashDetected();
  String getCrashReport();
  void saveCrashReport();
};

// Performance logger
class PerformanceLogger {
private:
  unsigned long loopCount;
  unsigned long maxLoopTime;
  unsigned long avgLoopTime;
  unsigned long lastLoopTime;
  unsigned long totalLoopTime;
  
  unsigned long freeMemory;
  unsigned long minFreeMemory;
  unsigned long maxMemoryUsage;
  
  unsigned long sensorUpdateCount;
  unsigned long communicationCount;
  unsigned long errorCount;
  
public:
  PerformanceLogger();
  void initialize();
  
  void logLoopTime(unsigned long loopTime);
  void logMemoryUsage();
  void logSensorUpdate();
  void logCommunication();
  void logError();
  
  // Getters
  unsigned long getLoopCount();
  unsigned long getMaxLoopTime();
  unsigned long getAvgLoopTime();
  unsigned long getFreeMemory();
  unsigned long getMinFreeMemory();
  
  // Report functions
  String getPerformanceReport();
  void resetStatistics();
};

// Configuration logger
class ConfigLogger {
private:
  String configFile;
  JsonDocument configDoc;
  
public:
  ConfigLogger();
  bool initialize();
  
  bool saveConfiguration();
  bool loadConfiguration();
  
  void setConfigValue(const String& key, const String& value);
  void setConfigValue(const String& key, float value);
  void setConfigValue(const String& key, int value);
  void setConfigValue(const String& key, bool value);
  
  String getConfigString(const String& key, const String& defaultValue = "");
  float getConfigFloat(const String& key, float defaultValue = 0.0);
  int getConfigInt(const String& key, int defaultValue = 0);
  bool getConfigBool(const String& key, bool defaultValue = false);
  
  void resetToDefaults();
  String exportConfig();
};

// Global logger instances
extern Logger logger;
extern FlightDataLogger flightLogger;
extern BlackBoxLogger blackBoxLogger;
extern PerformanceLogger performanceLogger;
extern ConfigLogger configLogger;

// Global logging functions (simplified interface)
void logEvent(const String& module, const String& message);
void logDebug(const String& module, const String& message);
void logInfo(const String& module, const String& message);
void logWarning(const String& module, const String& message);
void logError(const String& module, const String& message);
void logCritical(const String& module, const String& message);

// Initialization function
void initializeLoggers();

// Utility functions
String formatBytes(size_t bytes);
String getSystemInfo();
void dumpSystemInfo();

// Log level names
extern const char* LOG_LEVEL_NAMES[];
extern const char* LOG_CATEGORY_NAMES[];

#endif // LOGGER_H
