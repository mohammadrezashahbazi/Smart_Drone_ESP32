/*
 * Safety System for Smart Drone ESP32
 * Critical safety monitoring and emergency protocols
 */

#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>
#include "sensors.h"

// Safety thresholds
#define BATTERY_LOW_THRESHOLD 10.5      // Volts
#define BATTERY_CRITICAL_THRESHOLD 9.9  // Volts
#define MAX_ALTITUDE 100.0              // Meters
#define MIN_ALTITUDE 0.5                // Meters
#define MAX_TILT_ANGLE 45.0             // Degrees
#define MAX_VELOCITY 10.0               // m/s
#define GPS_TIMEOUT 10000               // milliseconds
#define SENSOR_TIMEOUT 5000             // milliseconds
#define EMERGENCY_DESCENT_RATE 1.0      // m/s

// Safety levels
enum SafetyLevel {
  SAFE = 0,
  WARNING = 1,
  CRITICAL = 2,
  EMERGENCY = 3
};

// Emergency types
enum EmergencyType {
  NONE = 0,
  BATTERY_CRITICAL,
  ALTITUDE_EXCEEDED,
  TILT_EXCEEDED,
  GPS_LOST,
  SENSOR_FAILURE,
  MANUAL_EMERGENCY,
  GEOFENCE_VIOLATION,
  COMMUNICATION_LOST,
  MOTOR_FAILURE
};

// Safety events
struct SafetyEvent {
  EmergencyType type;
  SafetyLevel level;
  String description;
  unsigned long timestamp;
  bool resolved;
};

// Geofence structure
struct Geofence {
  double centerLat;
  double centerLon;
  float radius;          // meters
  float maxAltitude;     // meters
  bool enabled;
};

// Safety system class
class SafetySystem {
private:
  SafetyLevel currentLevel;
  EmergencyType currentEmergency;
  
  // Safety counters
  unsigned long batteryLowStart;
  unsigned long gpsLostStart;
  unsigned long sensorFailureStart;
  unsigned long lastSafetyCheck;
  
  // Emergency flags
  bool emergencyLanding;
  bool returnToHome;
  bool motorsDisabled;
  bool safetyOverride;
  
  // Geofence
  Geofence geofence;
  
  // Safety events log
  SafetyEvent safetyEvents[50];
  int eventCount;
  
  // Internal methods
  void checkBatteryLevel();
  void checkAltitudeLimits();
  void checkTiltLimits();
  void checkGPSStatus();
  void checkSensorHealth();
  void checkGeofence();
  void checkCommunication();
  void processEmergency(EmergencyType type, SafetyLevel level, String description);
  void logSafetyEvent(EmergencyType type, SafetyLevel level, String description);
  
public:
  SafetySystem();
  void initialize();
  void update();
  
  // Safety checks
  bool performSafetyCheck();
  bool isFlightSafe();
  bool canTakeoff();
  bool canLand();
  
  // Emergency handling
  void triggerEmergency(EmergencyType type, String description = "");
  void triggerEmergencyLanding();
  void triggerReturnToHome();
  void resetEmergency();
  
  // Geofence management
  void setGeofence(double lat, double lon, float radius, float maxAlt);
  void enableGeofence(bool enable);
  bool isInsideGeofence();
  
  // Safety overrides
  void setSafetyOverride(bool enable);
  bool getSafetyOverride();
  
  // Status getters
  SafetyLevel getSafetyLevel();
  EmergencyType getCurrentEmergency();
  String getSafetyStatus();
  String getEmergencyDescription();
  bool isEmergencyLanding();
  bool isReturnToHome();
  bool areMotorsDisabled();
  
  // Safety events
  int getEventCount();
  SafetyEvent getEvent(int index);
  void clearEvents();
  
  // Recovery procedures
  void executeEmergencyLanding();
  void executeReturnToHome();
  void executeMotorShutdown();
  
  // Failsafe procedures
  void activateFailsafe();
  void deactivateFailsafe();
  bool isFailsafeActive();
};

// Flight envelope monitoring
class FlightEnvelope {
private:
  float maxRollAngle;
  float maxPitchAngle;
  float maxYawRate;
  float maxVerticalVelocity;
  float maxHorizontalVelocity;
  
public:
  FlightEnvelope();
  void setLimits(float roll, float pitch, float yaw, float vertVel, float horizVel);
  bool checkLimits(SensorData& data);
  bool isWithinEnvelope(float roll, float pitch, float yaw, float vertVel, float horizVel);
};

// Emergency landing controller
class EmergencyLanding {
private:
  bool active;
  float targetDescentRate;
  float currentAltitude;
  unsigned long startTime;
  
public:
  EmergencyLanding();
  void activate();
  void deactivate();
  bool isActive();
  void update(float currentAlt);
  float getDescentRate();
  bool isLandingComplete();
};

// Return to home controller
class ReturnToHome {
private:
  bool active;
  double homeLat;
  double homeLon;
  float homeAltitude;
  float returnAltitude;
  unsigned long startTime;
  
public:
  ReturnToHome();
  void setHome(double lat, double lon, float alt);
  void activate();
  void deactivate();
  bool isActive();
  void update(double currentLat, double currentLon, float currentAlt);
  bool isHomeReached();
  float getDistanceToHome();
  float getBearingToHome();
};

// Global safety system instance
extern SafetySystem safetySystem;
extern FlightEnvelope flightEnvelope;
extern EmergencyLanding emergencyLanding;
extern ReturnToHome returnToHome;

// Global safety functions
void initializeSafetySystem();
void updateSafetySystem();
bool isFlightSafe();
void triggerEmergency(EmergencyType type, String description = "");
void activateEmergencyLanding();
void activateReturnToHome();
void setGeofence(double lat, double lon, float radius, float maxAlt);
String getSafetyStatus();

// Safety constants
extern const float SAFETY_BATTERY_LOW;
extern const float SAFETY_BATTERY_CRITICAL;
extern const float SAFETY_MAX_ALTITUDE;
extern const float SAFETY_MAX_TILT;
extern const unsigned long SAFETY_GPS_TIMEOUT;

#endif
