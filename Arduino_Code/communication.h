#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "config.h"

// ساختار داده‌های ارتباطی
struct TelemetryData {
  float roll;
  float pitch;
  float yaw;
  float altitude;
  float latitude;
  float longitude;
  float batteryVoltage;
  int flightMode;
  bool isArmed;
  unsigned long timestamp;
};

struct RemoteControlData {
  float roll;
  float pitch;
  float yaw;
  float throttle;
  bool armSwitch;
  bool modeSwitch;
  bool killSwitch;
  bool isValid;
  unsigned long timestamp;
};

// کلاس ارتباطات
class CommunicationManager {
private:
  // WiFi و WebServer
  WebServer server;
  String ssid;
  String password;
  bool wifiConnected;
  
  // Serial Communication
  SoftwareSerial rcSerial;
  
  // داده‌های ارتباطی
  TelemetryData telemetryData;
  RemoteControlData rcData;
  
  // متغیرهای زمانی
  unsigned long lastTelemetryTime;
  unsigned long lastRCTime;
  unsigned long lastHeartbeat;
  
  // متدهای خصوصی
  void setupWiFi();
  void setupWebServer();
  void handleRoot();
  void handleTelemetry();
  void handleControl();
  void handleCORS();
  bool parseRCData(String data);
  String createTelemetryJSON();
  
public:
  CommunicationManager();
  
  // راه‌اندازی
  bool initialize();
  
  // به‌روزرسانی
  void update();
  
  // ارسال و دریافت داده
  void sendTelemetry(const TelemetryData& data);
  RemoteControlData getRemoteControlData();
  
  // وضعیت ارتباط
  bool isRCConnected();
  bool isWiFiConnected();
  
  // تنظیمات
  void setWiFiCredentials(String ssid, String password);
  void setTelemetryRate(int rate);
  
  // Debug
  void printStatus();
};

// متغیرهای سراسری
extern CommunicationManager commManager;
extern RemoteControlData currentRCData;
extern TelemetryData currentTelemetry;

// توابع کمکی سراسری
void initializeCommunication();
void updateCommunication();
bool isRemoteControlActive();
RemoteControlData getRemoteControlInput();
void sendDroneTelemetry(const TelemetryData& data);

// ثابت‌ها
#define RC_TIMEOUT 1000        // ms
#define TELEMETRY_RATE 10      // Hz
#define WIFI_TIMEOUT 10000     // ms
#define HEARTBEAT_INTERVAL 500 // ms

// پین‌های ارتباطی
#define RC_RX_PIN 16
#define RC_TX_PIN 17
#define RC_BAUD_RATE 9600

// WiFi Settings
#define AP_SSID "DroneControl"
#define AP_PASSWORD "12345678"
#define WEB_SERVER_PORT 80

#endif
