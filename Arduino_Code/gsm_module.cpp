/*
 * GSM Module Implementation for Smart Drone ESP32
 */

#include "gsm_module.h"
#include "logger.h"

// Global instance
GSMModule gsmModule;

GSMModule::GSMModule() : gsmSerial(GSM_RX_PIN, GSM_TX_PIN) {
  isModuleReady = false;
  authorizedCount = 0;
  
  // Default authorized numbers (change these!)
  addAuthorizedNumber("+989123456789");
  addAuthorizedNumber("+989987654321");
}

bool GSMModule::initialize() {
  Serial.println("Initializing GSM module...");
  
  // Power on module
  powerOn();
  delay(5000);
  
  // Initialize serial communication
  gsmSerial.begin(9600);
  delay(1000);
  
  // Test AT commands
  gsmSerial.println("AT");
  delay(1000);
  
  if (gsmSerial.find("OK")) {
    Serial.println("GSM module responded");
    
    // Set SMS text mode
    gsmSerial.println("AT+CMGF=1");
    delay(1000);
    
    // Set SMS notification
    gsmSerial.println("AT+CNMI=2,2,0,0,0");
    delay(1000);
    
    // Check network registration
    if (isNetworkConnected()) {
      isModuleReady = true;
      Serial.println("GSM module initialized successfully");
      logEvent("GSM_INIT", "GSM module ready");
      return true;
    }
  }
  
  Serial.println("GSM module initialization failed");
  return false;
}

bool GSMModule::sendSMS(String number, String message) {
  if (!isModuleReady) return false;
  
  gsmSerial.println("AT+CMGS=\"" + number + "\"");
  delay(1000);
  
  gsmSerial.print(message);
  delay(1000);
  
  gsmSerial.write(26); // Ctrl+Z
  delay(5000);
  
  if (gsmSerial.find("OK")) {
    Serial.println("SMS sent successfully");
    logEvent("SMS_SENT", "SMS sent to " + number);
    return true;
  }
  
  Serial.println("SMS send failed");
  return false;
}

bool GSMModule::checkSMS() {
  if (!isModuleReady) return false;
  
  // Check for new SMS
  if (gsmSerial.available()) {
    String response = gsmSerial.readString();
    
    if (response.indexOf("+CMT:") > -1) {
      // Parse SMS
      int firstQuote = response.indexOf("\"");
      int secondQuote = response.indexOf("\"", firstQuote + 1);
      String senderNumber = response.substring(firstQuote + 1, secondQuote);
      
      int messageStart = response.indexOf("\n", response.indexOf("+CMT:")) + 1;
      String message = response.substring(messageStart);
      message.trim();
      
      Serial.println("SMS received from: " + senderNumber);
      Serial.println("Message: " + message);
      
      // Process command if from authorized number
      if (isAuthorized(senderNumber)) {
        SMSCommand cmd = parseSMSCommand(message);
        handleSMSCommand(cmd, senderNumber);
      } else {
        Serial.println("Unauthorized number: " + senderNumber);
        sendSMS(senderNumber, "⚠️ Unauthorized access attempt!");
      }
      
      return true;
    }
  }
  
  return false;
}

SMSCommand GSMModule::parseSMSCommand(String message) {
  message.toUpperCase();
  message.trim();
  
  if (message == "STATUS" || message == "STAT") return CMD_STATUS;
  if (message == "ARM") return CMD_ARM;
  if (message == "TAKEOFF" || message == "TO") return CMD_TAKEOFF;
  if (message == "LAND") return CMD_LAND;
  if (message == "EMERGENCY" || message == "EMRG") return CMD_EMERGENCY;
  if (message == "LOCATION" || message == "LOC") return CMD_LOCATION;
  if (message == "BATTERY" || message == "BAT") return CMD_BATTERY;
  if (message == "RTH" || message == "HOME") return CMD_RTH;
  if (message.startsWith("MISSION")) return CMD_MISSION;
  
  return CMD_UNKNOWN;
}

void GSMModule::handleSMSCommand(SMSCommand cmd, String senderNumber) {
  switch (cmd) {
    case CMD_STATUS:
      sendStatusSMS(senderNumber);
      break;
      
    case CMD_ARM:
      if (currentMode == STANDBY) {
        armDrone();
        sendSMS(senderNumber, "🚁 Drone ARMED");
      } else {
        sendSMS(senderNumber, "❌ Cannot ARM - not in standby");
      }
      break;
      
    case CMD_TAKEOFF:
      if (currentMode == ARM) {
        takeoff();
        sendSMS(senderNumber, "🚁 Takeoff initiated");
      } else {
        sendSMS(senderNumber, "❌ Cannot takeoff - not armed");
      }
      break;
      
    case CMD_LAND:
      if (isFlying) {
        land();
        sendSMS(senderNumber, "🚁 Landing initiated");
      } else {
        sendSMS(senderNumber, "❌ Not flying");
      }
      break;
      
    case CMD_EMERGENCY:
      emergencyLand();
      sendSMS(senderNumber, "🚨 EMERGENCY LANDING ACTIVATED");
      break;
      
    case CMD_LOCATION:
      sendLocationSMS(senderNumber);
      break;
      
    case CMD_BATTERY:
      sendBatterySMS(senderNumber);
      break;
      
    case CMD_RTH:
      if (isFlying) {
        currentMode = RTH;
        sendSMS(senderNumber, "🏠 Return to home activated");
      } else {
        sendSMS(senderNumber, "❌ Not flying");
      }
      break;
      
    default:
      sendSMS(senderNumber, "❓ Unknown command. Send STATUS for help");
      break;
  }
}

bool GSMModule::isAuthorized(String number) {
  for (int i = 0; i < authorizedCount; i++) {
    if (authorizedNumbers[i] == number) {
      return true;
    }
  }
  return false;
}

void GSMModule::addAuthorizedNumber(String number) {
  if (authorizedCount < 5) {
    authorizedNumbers[authorizedCount] = number;
    authorizedCount++;
  }
}

void GSMModule::sendStatusSMS(String number) {
  String status = "🚁 DRONE STATUS\n";
  status += "Mode: " + String(currentMode) + "\n";
  status += "Armed: " + String(isArmed ? "YES" : "NO") + "\n";
  status += "Flying: " + String(isFlying ? "YES" : "NO") + "\n";
  status += "Battery: " + String(batteryVoltage) + "V\n";
  status += "Altitude: " + String(altitude) + "m\n";
  status += "Signal: " + getSignalStrength();
  
  sendSMS(number, status);
}

void GSMModule::sendLocationSMS(String number) {
  String location = "📍 DRONE LOCATION\n";
  location += "Lat: " + String(latitude, 6) + "\n";
  location += "Lon: " + String(longitude, 6) + "\n";
  location += "Alt: " + String(altitude) + "m\n";
  location += "Google Maps: https://maps.google.com/?q=" + String(latitude, 6) + "," + String(longitude, 6);
  
  sendSMS(number, location);
}

void GSMModule::sendBatterySMS(String number) {
  String battery = "🔋 BATTERY STATUS\n";
  battery += "Voltage: " + String(batteryVoltage) + "V\n";
  
  if (batteryVoltage > 11.5) {
    battery += "Level: HIGH ✅";
  } else if (batteryVoltage > 10.5) {
    battery += "Level: MEDIUM ⚠️";
  } else {
    battery += "Level: LOW ❌";
  }
  
  sendSMS(number, battery);
}

void GSMModule::sendEmergencyAlert() {
  String alert = "🚨 DRONE EMERGENCY ALERT\n";
  alert += "Time: " + String(millis()) + "\n";
  alert += "Location: " + String(latitude, 6) + "," + String(longitude, 6) + "\n";
  alert += "Battery: " + String(batteryVoltage) + "V\n";
  alert += "Emergency landing activated!";
  
  // Send to all authorized numbers
  for (int i = 0; i < authorizedCount; i++) {
    sendSMS(authorizedNumbers[i], alert);
  }
}

bool GSMModule::isNetworkConnected() {
  gsmSerial.println("AT+CREG?");
  delay(1000);
  
  String response = gsmSerial.readString();
  return (response.indexOf("+CREG: 0,1") > -1 || response.indexOf("+CREG: 0,5") > -1);
}

String GSMModule::getSignalStrength() {
  gsmSerial.println("AT+CSQ");
  delay(1000);
  
  String response = gsmSerial.readString();
  int start = response.indexOf("+CSQ: ") + 6;
  int end = response.indexOf(",", start);
  
  if (start > 5 && end > start) {
    int signal = response.substring(start, end).toInt();
    return String(signal) + "/31";
  }
  
  return "Unknown";
}

void GSMModule::powerOn() {
  pinMode(GSM_POWER_PIN, OUTPUT);
  digitalWrite(GSM_POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(GSM_POWER_PIN, LOW);
}

void GSMModule::powerOff() {
  digitalWrite(GSM_POWER_PIN, HIGH);
  delay(3000);
  digitalWrite(GSM_POWER_PIN, LOW);
}

void GSMModule::reset() {
  gsmSerial.println("AT+CFUN=1,1");
  delay(10000);
}

// Global functions
void initializeGSM() {
  gsmModule.initialize();
}

void handleGSMCommands() {
  gsmModule.checkSMS();
}

void sendEmergencyNotification(String message) {
  gsmModule.sendEmergencyAlert();
}

void sendTelemetryViaSMS() {
  // Send telemetry to first authorized number
  if (gsmModule.authorizedCount > 0) {
    gsmModule.sendStatusSMS(gsmModule.authorizedNumbers[0]);
  }
}
