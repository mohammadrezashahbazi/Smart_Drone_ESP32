/*
 * Sensors Module for Smart Drone ESP32
 * Handles all sensor readings and data processing
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Sensor pins
#define GPS_RX_PIN 12
#define GPS_TX_PIN 13
#define BATTERY_PIN A0
#define ALTITUDE_TRIGGER_PIN 14
#define ALTITUDE_ECHO_PIN 15
#define TEMPERATURE_PIN 22
#define COMPASS_ADDRESS 0x1E

// Data structures
struct SensorData {
  // IMU data
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  
  // GPS data
  double latitude, longitude;
  float gpsAltitude;
  int satellites;
  bool gpsValid;
  
  // Other sensors
  float batteryVoltage;
  float ultrasonic_altitude;
  float compass_heading;
  float external_temperature;
  
  // Calculated values
  float roll, pitch, yaw;
  float velocity_x, velocity_y, velocity_z;
  
  // Timestamps
  unsigned long timestamp;
  unsigned long last_gps_update;
};

// Sensor classes
class IMUSensor {
private:
  Adafruit_MPU6050 mpu;
  bool initialized;
  
public:
  IMUSensor();
  bool initialize();
  bool readData(SensorData& data);
  void calibrate();
  bool isHealthy();
  void calculateOrientation(SensorData& data);
};

class GPSSensor {
private:
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial;
  bool initialized;
  
public:
  GPSSensor();
  bool initialize();
  bool readData(SensorData& data);
  bool isHealthy();
  int getSatelliteCount();
  bool hasValidFix();
};

class BatterySensor {
private:
  float voltage_divider_ratio;
  float calibration_factor;
  
public:
  BatterySensor();
  bool initialize();
  float readVoltage();
  int getBatteryPercentage();
  bool isLowBattery();
  bool isCriticalBattery();
};

class AltitudeSensor {
private:
  bool initialized;
  float calibration_offset;
  
public:
  AltitudeSensor();
  bool initialize();
  float readAltitude();
  void calibrateGround();
  bool isHealthy();
};

class CompassSensor {
private:
  bool initialized;
  float declination_angle;
  
public:
  CompassSensor();
  bool initialize();
  float readHeading();
  void calibrate();
  bool isHealthy();
};

class TemperatureSensor {
private:
  OneWire oneWire;
  DallasTemperature sensors;
  bool initialized;
  
public:
  TemperatureSensor();
  bool initialize();
  float readTemperature();
  bool isHealthy();
};

// Sensor manager class
class SensorManager {
private:
  IMUSensor imuSensor;
  GPSSensor gpsSensor;
  BatterySensor batterySensor;
  AltitudeSensor altitudeSensor;
  CompassSensor compassSensor;
  TemperatureSensor temperatureSensor;
  
  SensorData currentData;
  SensorData previousData;
  
  unsigned long lastUpdate;
  bool sensorsInitialized;
  
public:
  SensorManager();
  bool initialize();
  void updateAllSensors();
  SensorData getCurrentData();
  SensorData getPreviousData();
  bool isDataValid();
  void calibrateAllSensors();
  String getHealthStatus();
  void logSensorData();
  
  // Individual sensor access
  IMUSensor& getIMU() { return imuSensor; }
  GPSSensor& getGPS() { return gpsSensor; }
  BatterySensor& getBattery() { return batterySensor; }
  AltitudeSensor& getAltitude() { return altitudeSensor; }
  CompassSensor& getCompass() { return compassSensor; }
  TemperatureSensor& getTemperature() { return temperatureSensor; }
};

// Global sensor manager
extern SensorManager sensorManager;

// Global variables (for main.ino compatibility)
extern float batteryVoltage;
extern float altitude;
extern double latitude;
extern double longitude;

// Function declarations
void initializeSensors();
void updateSensors();
bool safetyCheck();
void calibrateSensors();
String getSensorStatus();
void logSensorReadings();

// Utility functions
float calculateDistance(double lat1, double lon1, double lat2, double lon2);
float calculateBearing(double lat1, double lon1, double lat2, double lon2);
bool isValidGPSCoordinate(double coordinate);
float lowPassFilter(float current, float previous, float alpha);
float complementaryFilter(float gyro, float accel, float dt, float alpha);

#endif
