/*
 * Safety System Implementation for Smart Drone ESP32
 */

#include "safety_system.h"
#include "logger.h"
#include <math.h>

// Global instances
SafetySystem safetySystem;
FlightEnvelope flightEnvelope;
EmergencyLanding emergencyLanding;
ReturnToHome returnToHome;

// Safety constants
const float SAFETY_BATTERY_LOW = 10.5;
const float SAFETY_BATTERY_CRITICAL = 9.9;
const float SAFETY_MAX_ALTITUDE = 100.0;
const float SAFETY_MAX_TILT = 45.0;
const unsigned long SAFETY_GPS_TIMEOUT = 10000;

// SafetySystem Implementation
SafetySystem::SafetySystem() {
  currentLevel = SAFE;
  currentEmergency = NONE;
  
  batteryLowStart = 0;
  gpsLostStart = 0;
  sensorFailureStart = 0;
  lastSafetyCheck = 0;
  
  emergencyLanding = false;
  returnToHome = false;
  motorsDisabled = false;
  safetyOverride = false;
  
  eventCount = 0;
  
  // Initialize geofence
  geofence.enabled = false;
  geofence.centerLat = 0.0;
  geofence.centerLon = 0.0;
  geofence.radius = 100.0;
  geofence.maxAltitude = 50.0;
}

void SafetySystem::initialize() {
  Serial.println("Initializing Safety System...");
  
  currentLevel = SAFE;
  currentEmergency = NONE;
  
  // Reset all flags
  emergencyLanding = false;
  returnToHome = false;
  motorsDisabled = false;
  
  // Clear events
  eventCount = 0;
  
  Serial.println("Safety System initialized");
  logEvent("SAFETY", "Safety system initialized");
}

void SafetySystem::update() {
  if (millis() - lastSafetyCheck < 100) return; // Check every 100ms
  
  lastSafetyCheck = millis();
  
  // Perform all safety checks
  checkBatteryLevel();
  checkAltitudeLimits();
  checkTiltLimits();
  checkGPSStatus();
  checkSensorHealth();
  checkGeofence();
  checkCommunication();
  
  // Update emergency procedures
  if (emergencyLanding) {
    executeEmergencyLanding();
  }
  
  if (returnToHome) {
    executeReturnToHome();
  }
}

void SafetySystem::checkBatteryLevel() {
  SensorData data = sensorManager.getCurrentData();
  
  if (data.batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
    if (currentLevel != EMERGENCY) {
      processEmergency(BATTERY_CRITICAL, EMERGENCY, "Critical battery voltage: " + String(data.batteryVoltage) + "V");
      triggerEmergencyLanding();
    }
  } else if (data.batteryVoltage < BATTERY_LOW_THRESHOLD) {
    if (currentLevel == SAFE) {
      processEmergency(BATTERY_CRITICAL, WARNING, "Low battery voltage: " + String(data.batteryVoltage) + "V");
      batteryLowStart = millis();
    }
  } else {
    if (currentEmergency == BATTERY_CRITICAL && currentLevel == WARNING) {
      currentLevel = SAFE;
      currentEmergency = NONE;
      batteryLowStart = 0;
    }
  }
}

void SafetySystem::checkAltitudeLimits() {
  SensorData data = sensorManager.getCurrentData();
  
  if (data.ultrasonic_altitude > MAX_ALTITUDE) {
    processEmergency(ALTITUDE_EXCEEDED, CRITICAL, "Altitude exceeded: " + String(data.ultrasonic_altitude) + "m");
  } else if (data.ultrasonic_altitude < MIN_ALTITUDE && data.ultrasonic_altitude > 0) {
    processEmergency(ALTITUDE_EXCEEDED, WARNING, "Altitude too low: " + String(data.ultrasonic_altitude) + "m");
  }
}

void SafetySystem::checkTiltLimits() {
  SensorData data = sensorManager.getCurrentData();
  
  float maxTilt = max(abs(data.roll), abs(data.pitch));
  
  if (maxTilt > MAX_TILT_ANGLE) {
    processEmergency(TILT_EXCEEDED, CRITICAL, "Excessive tilt: " + String(maxTilt) + "°");
    triggerEmergencyLanding();
  }
}

void SafetySystem::checkGPSStatus() {
  SensorData data = sensorManager.getCurrentData();
  
  if (!data.gpsValid || (millis() - data.last_gps_update) > GPS_TIMEOUT) {
    if (gpsLostStart == 0) {
      gpsLostStart = millis();
    } else if (millis() - gpsLostStart > 5000) { // 5 seconds timeout
      processEmergency(GPS_LOST, WARNING, "GPS signal lost");
    }
  } else {
    if (currentEmergency == GPS_LOST) {
      currentLevel = SAFE;
      currentEmergency = NONE;
      gpsLostStart = 0;
    }
  }
}

void SafetySystem::checkSensorHealth() {
  if (!sensorManager.getIMU().isHealthy()) {
    processEmergency(SENSOR_FAILURE, CRITICAL, "IMU sensor failure");
    triggerEmergencyLanding();
  }
  
  if (!sensorManager.getAltitude().isHealthy()) {
    processEmergency(SENSOR_FAILURE, CRITICAL, "Altitude sensor failure");
    triggerEmergencyLanding();
  }
}

void SafetySystem::checkGeofence() {
  if (!geofence.enabled) return;
  
  SensorData data = sensorManager.getCurrentData();
  
  if (data.gpsValid) {
    float distance = calculateDistance(data.latitude, data.longitude, geofence.centerLat, geofence.centerLon);
    
    if (distance > geofence.radius) {
      processEmergency(GEOFENCE_VIOLATION, CRITICAL, "Geofence violation: " + String(distance) + "m from center");
      triggerReturnToHome();
    }
    
    if (data.ultrasonic_altitude > geofence.maxAltitude) {
      processEmergency(GEOFENCE_VIOLATION, CRITICAL, "Altitude geofence violation: " + String(data.ultrasonic_altitude) + "m");
    }
  }
}

void SafetySystem::checkCommunication() {
  // Check for communication timeout
  // This would be implemented based on your communication protocol
}

void SafetySystem::processEmergency(EmergencyType type, SafetyLevel level, String description) {
  if (level > currentLevel) {
    currentLevel = level;
    currentEmergency = type;
    
    logSafetyEvent(type, level, description);
    
    Serial.println("SAFETY ALERT [" + String(level) + "]: " + description);
    
    // Send SMS alert if GSM is available
    if (level >= CRITICAL) {
      // Send emergency SMS
      String smsMessage = "DRONE EMERGENCY: " + description;
      // gsm.sendSMS(emergencyNumber, smsMessage);
    }
  }
}

void SafetySystem::logSafetyEvent(EmergencyType type, SafetyLevel level, String description) {
  if (eventCount < 50) {
    safetyEvents[eventCount].type = type;
    safetyEvents[eventCount].level = level;
    safetyEvents[eventCount].description = description;
    safetyEvents[eventCount].timestamp = millis();
    safetyEvents[eventCount].resolved = false;
    eventCount++;
  }
  
  // Log to file
  String logData = "SAFETY_EVENT," + String(type) + "," + String(level) + "," + description;
  logEvent("SAFETY", logData);
}

bool SafetySystem::performSafetyCheck() {
  update();
  return currentLevel <= WARNING && !safetyOverride;
}

bool SafetySystem::isFlightSafe() {
  SensorData data = sensorManager.getCurrentData();
  
  // Battery check
  if (data.batteryVoltage < BATTERY_LOW_THRESHOLD) return false;
  
  // Sensor health check
  if (!sensorManager.getIMU().isHealthy()) return false;
  if (!sensorManager.getAltitude().isHealthy()) return false;
  
  // GPS check (if required)
  if (!data.gpsValid && geofence.enabled) return false;
  
  return currentLevel <= WARNING;
}

bool SafetySystem::canTakeoff() {
  if (!isFlightSafe()) return false;
  
  SensorData data = sensorManager.getCurrentData();
  
  // Check if drone is level
  if (abs(data.roll) > 5.0 || abs(data.pitch) > 5.0) return false;
  
  // Check battery
  if (data.batteryVoltage < 11.0) return false;
  
  // Check altitude sensor
  if (data.ultrasonic_altitude < 0) return false;
  
  return true;
}

bool SafetySystem::canLand() {
  SensorData data = sensorManager.getCurrentData();
  
  // Check if close to ground
  if (data.ultrasonic_altitude < 0.3) return true;
  
  // Check if descent rate is safe
  // Implementation depends on velocity calculation
  
  return true;
}

void SafetySystem::triggerEmergency(EmergencyType type, String description) {
  processEmergency(type, EMERGENCY, description);
  
  switch (type) {
    case BATTERY_CRITICAL:
    case ALTITUDE_EXCEEDED:
    case TILT_EXCEEDED:
    case SENSOR_FAILURE:
      triggerEmergencyLanding();
      break;
      
    case GPS_LOST:
    case GEOFENCE_VIOLATION:
      triggerReturnToHome();
      break;
      
    case MANUAL_EMERGENCY:
      triggerEmergencyLanding();
      break;
      
    default:
      break;
  }
}

void SafetySystem::triggerEmergencyLanding() {
  if (!emergencyLanding) {
    emergencyLanding = true;
    emergencyLanding.activate();
    
    Serial.println("EMERGENCY LANDING ACTIVATED");
    logEvent("SAFETY", "Emergency landing activated");
  }
}

void SafetySystem::triggerReturnToHome() {
  if (!returnToHome) {
    returnToHome = true;
    returnToHome.activate();
    
    Serial.println("RETURN TO HOME ACTIVATED");
    logEvent("SAFETY", "Return to home activated");
  }
}

void SafetySystem::executeEmergencyLanding() {
  if (emergencyLanding.isActive()) {
    SensorData data = sensorManager.getCurrentData();
    emergencyLanding.update(data.ultrasonic_altitude);
    
    if (emergencyLanding.isLandingComplete()) {
      emergencyLanding = false;
      emergencyLanding.deactivate();
      motorsDisabled = true;
      
      Serial.println("EMERGENCY LANDING COMPLETE");
      logEvent("SAFETY", "Emergency landing complete");
    }
  }
}

void SafetySystem::executeReturnToHome() {
  if (returnToHome.isActive()) {
    SensorData data = sensorManager.getCurrentData();
    returnToHome.update(data.latitude, data.longitude, data.ultrasonic_altitude);
    
    if (returnToHome.isHomeReached()) {
      returnToHome = false;
      returnToHome.deactivate();
      triggerEmergencyLanding();
      
      Serial.println("RETURN TO HOME COMPLETE");
      logEvent("SAFETY", "Return to home complete");
    }
  }
}

void SafetySystem::setGeofence(double lat, double lon, float radius, float maxAlt) {
  geofence.centerLat = lat;
  geofence.centerLon = lon;
  geofence.radius = radius;
  geofence.maxAltitude = maxAlt;
  geofence.enabled = true;
  
  Serial.println("Geofence set: " + String(lat, 6) + "," + String(lon, 6) + " R=" + String(radius) + "m");
}

void SafetySystem::enableGeofence(bool enable) {
  geofence.enabled = enable;
}

bool SafetySystem::isInsideGeofence() {
  if (!geofence.enabled) return true;
  
  SensorData data = sensorManager.getCurrentData();
  
  if (!data.gpsValid) return false;
  
  float distance = calculateDistance(data.latitude, data.longitude, geofence.centerLat, geofence.centerLon);
  
  return distance <= geofence.radius && data.ultrasonic_altitude <= geofence.maxAltitude;
}

SafetyLevel SafetySystem::getSafetyLevel() {
  return currentLevel;
}

EmergencyType SafetySystem::getCurrentEmergency() {
  return currentEmergency;
}

String SafetySystem::getSafetyStatus() {
  String status = "Safety Status: ";
  
  switch (currentLevel) {
    case SAFE:
      status += "SAFE";
      break;
    case WARNING:
      status += "WARNING";
      break;
    case CRITICAL:
      status += "CRITICAL";
      break;
    case EMERGENCY:
      status += "EMERGENCY";
      break;
  }
  
  if (currentEmergency != NONE) {
    status += " - " + getEmergencyDescription();
  }
  
  return status;
}

String SafetySystem::getEmergencyDescription() {
  switch (currentEmergency) {
    case BATTERY_CRITICAL: return "Battery Critical";
    case ALTITUDE_EXCEEDED: return "Altitude Exceeded";
    case TILT_EXCEEDED: return "Tilt Exceeded";
    case GPS_LOST: return "GPS Lost";
    case SENSOR_FAILURE: return "Sensor Failure";
    case MANUAL_EMERGENCY: return "Manual Emergency";
    case GEOFENCE_VIOLATION: return "Geofence Violation";
    case COMMUNICATION_LOST: return "Communication Lost";
    case MOTOR_FAILURE: return "Motor Failure";
    default: return "None";
  }
}

// FlightEnvelope Implementation
FlightEnvelope::FlightEnvelope() {
  maxRollAngle = 30.0;
  maxPitchAngle = 30.0;
  maxYawRate = 90.0;
  maxVerticalVelocity = 5.0;
  maxHorizontalVelocity = 10.0;
}

void FlightEnvelope::setLimits(float roll, float pitch, float yaw, float vertVel, float horizVel) {
  maxRollAngle = roll;
  maxPitchAngle = pitch;
  maxYawRate = yaw;
  maxVerticalVelocity = vertVel;
  maxHorizontalVelocity = horizVel;
}

bool FlightEnvelope::checkLimits(SensorData& data) {
  if (abs(data.roll) > maxRollAngle) return false;
  if (abs(data.pitch) > maxPitchAngle) return false;
  if (abs(data.gyroZ) > maxYawRate) return false;
  
  // Velocity checks would require velocity calculation
  
  return true;
}

// EmergencyLanding Implementation
EmergencyLanding::EmergencyLanding() {
  active = false;
  targetDescentRate = EMERGENCY_DESCENT_RATE;
  startTime = 0;
}

void EmergencyLanding::activate() {
  active = true;
  startTime = millis();
  currentAltitude = sensorManager.getCurrentData().ultrasonic_altitude;
  
  Serial.println("Emergency landing activated");
}

void EmergencyLanding::deactivate() {
  active = false;
  startTime = 0;
}

bool EmergencyLanding::isActive() {
  return active;
}

void EmergencyLanding::update(float currentAlt) {
  if (!active) return;
  
  currentAltitude = currentAlt;
  
  // Control descent rate here
  // This would interface with motor control
}

bool EmergencyLanding::isLandingComplete() {
  return currentAltitude < 0.2; // 20cm from ground
}

// ReturnToHome Implementation
ReturnToHome::ReturnToHome() {
  active = false;
  homeLat = 0.0;
  homeLon = 0.0;
  homeAltitude = 0.0;
  returnAltitude = 10.0;
  startTime = 0;
}

void ReturnToHome::setHome(double lat, double lon, float alt) {
  homeLat = lat;
  homeLon = lon;
  homeAltitude = alt;
}

void ReturnToHome::activate() {
  active = true;
  startTime = millis();
  
  Serial.println("Return to home activated");
}

void ReturnToHome::deactivate() {
  active = false;
  startTime = 0;
}

bool ReturnToHome::isActive() {
  return active;
}

void ReturnToHome::update(double currentLat, double currentLon, float currentAlt) {
  if (!active) return;
  
  float distance = getDistanceToHome();
  
  if (distance < 2.0) { // Within 2 meters
    active = false;
  }
  
  // Navigation logic would go here
}

bool ReturnToHome::isHomeReached() {
  return getDistanceToHome() < 2.0;
}

float ReturnToHome::getDistanceToHome() {
  SensorData data = sensorManager.getCurrentData();
  return calculateDistance(data.latitude, data.longitude, homeLat, homeLon);
}

float ReturnToHome::getBearingToHome() {
  SensorData data = sensorManager.getCurrentData();
  return calculateBearing(data.latitude, data.longitude, homeLat, homeLon);
}

// Global functions
void initializeSafetySystem() {
  safetySystem.initialize();
}

void updateSafetySystem() {
  safetySystem.update();
}

bool isFlightSafe() {
  return safetySystem.isFlightSafe();
}

void triggerEmergency(EmergencyType type, String description) {
  safetySystem.triggerEmergency(type, description);
}

void activateEmergencyLanding() {
  safetySystem.triggerEmergencyLanding();
}

void activateReturnToHome() {
  safetySystem.triggerReturnToHome();
}

void setGeofence(double lat, double lon, float radius, float maxAlt) {
  safetySystem.setGeofence(lat, lon, radius, maxAlt);
}

String getSafetyStatus() {
  return safetySystem.getSafetyStatus();
}
