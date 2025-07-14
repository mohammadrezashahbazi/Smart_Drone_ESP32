/*
 * GSM Module for Smart Drone ESP32
 * Handles SMS commands and GSM communication
 */

#ifndef GSM_MODULE_H
#define GSM_MODULE_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// GSM module pins
#define GSM_RX_PIN 16
#define GSM_TX_PIN 17
#define GSM_POWER_PIN 23

// SMS command types
enum SMSCommand {
  CMD_STATUS,
  CMD_ARM,
  CMD_TAKEOFF,
  CMD_LAND,
  CMD_EMERGENCY,
  CMD_LOCATION,
  CMD_BATTERY,
  CMD_RTH,
  CMD_MISSION,
  CMD_UNKNOWN
};

// GSM module class
class GSMModule {
private:
  SoftwareSerial gsmSerial;
  bool isModuleReady;
  String authorizedNumbers[5];
  int authorizedCount;
  
public:
  GSMModule();
  bool initialize();
  bool sendSMS(String number, String message);
  bool checkSMS();
  SMSCommand parseSMSCommand(String message);
  void handleSMSCommand(SMSCommand cmd, String senderNumber);
  bool isAuthorized(String number);
  void addAuthorizedNumber(String number);
  void sendStatusSMS(String number);
  void sendLocationSMS(String number);
  void sendBatterySMS(String number);
  void sendEmergencyAlert();
  bool isNetworkConnected();
  String getSignalStrength();
  void powerOn();
  void powerOff();
  void reset();
};

// Global GSM instance
extern GSMModule gsmModule;

// Function declarations
void initializeGSM();
void handleGSMCommands();
void sendEmergencyNotification(String message);
void sendTelemetryViaSMS();

#endif
