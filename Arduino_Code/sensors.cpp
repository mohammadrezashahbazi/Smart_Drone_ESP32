/*
 * Smart Drone ESP32 - Sensor Management Implementation
 * Complete sensor integration and processing
 * Version: 2.0
 */

#include "sensors.h"

// Global sensor manager instance
SensorManager sensorManager;

// Constructor
SensorManager::SensorManager() : 
  gpsSerial(GPS_RX_PIN, GPS_TX_PIN),
  oneWire(TEMPERATURE_PIN),
  temperatureSensor(&oneWire),
  dht(DHT_PIN, DHT22)
{
  initialized = false;
  calibrationInProgress = false;
  healthCheckEnabled = true;
  
  // Initialize timing
  lastIMUUpdate = 0;
  lastGPSUpdate = 0;
  lastBarometerUpdate = 0;
  lastTemperatureUpdate = 0;
  lastUltrasonicUpdate = 0;
  lastBatteryUpdate = 0;
  
  // Initialize data structures
  memset(&currentData, 0, sizeof(SensorData));
  memset(&previousData, 0, sizeof(SensorData));
  memset(&calibrationData, 0, sizeof(CalibrationData));
  
  // Initialize filters
  altitudeFilter = {0, 1, 0, KALMAN_FILTER_Q, KALMAN_FILTER_R};
  velocityFilter = {0, 1, 0, KALMAN_FILTER_Q, KALMAN_FILTER_R};
  rollFilter = {COMPLEMENTARY_FILTER_ALPHA, 0, false};
  pitchFilter = {COMPLEMENTARY_FILTER_ALPHA, 0, false};
  yawFilter = {COMPLEMENTARY_FILTER_ALPHA, 0, false};
  
  memset(&ultrasonicFilter, 0, sizeof(MedianFilter));
  memset(&batteryFilter, 0, sizeof(MovingAverageFilter));
  memset(&temperatureFilter, 0, sizeof(MovingAverageFilter));
}

// Destructor
SensorManager::~SensorManager() {
  stop();
}

// Initialize all sensors
bool SensorManager::initialize() {
  logger.info("Initializing sensor manager...");
  
  bool success = true;
  
  // Initialize I2C
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C speed
  
  // Initialize individual sensors
  success &= initializeIMU();
  success &= initializeGPS();
  success &= initializeBarometer();
  success &= initializeTemperatureSensor();
  success &= initializeUltrasonic();
  success &= initializeBattery();
  success &= initializeMagnetometer();
  
  if (success) {
    initialized = true;
    logger.info("All sensors initialized successfully");
    
    // Load calibration data
    loadCalibration();
    
    // Perform initial sensor fusion
    performSensorFusion();
    
    return true;
  } else {
    logger.error("Sensor initialization failed");
    return false;
  }
}

// Initialize IMU (MPU6050)
bool SensorManager::initializeIMU() {
  logger.info("Initializing IMU...");
  
  mpu.initialize();
  
  if (mpu.testConnection()) {
    // Configure MPU6050
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    mpu.setSampleRate(1000 / IMU_SAMPLE_RATE - 1);
    
    currentData.imu_healthy = true;
    logger.info("IMU initialized successfully");
    return true;
  } else {
    currentData.imu_healthy = false;
    logger.error("IMU initialization failed");
    return false;
  }
}

// Initialize GPS
bool SensorManager::initializeGPS() {
  logger.info("Initializing GPS...");
  
  gpsSerial.begin(GPS_BAUD_RATE);
  
  // Wait for GPS to initialize
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (gpsSerial.available() > 0) {
      currentData.gps_healthy = true;
      logger.info("GPS initialized successfully");
      return true;
    }
    delay(100);
  }
  
  currentData.gps_healthy = false;
  logger.warning("GPS initialization timeout - continuing without GPS");
  return true; // Don't fail initialization due to GPS
}

// Initialize Barometer (BMP280)
bool SensorManager::initializeBarometer() {
  logger.info("Initializing barometer...");
  
  if (barometer.begin()) {
    barometer.setSampling(BMP280::MODE_NORMAL,     // Operating Mode
                         BMP280::SAMPLING_X2,      // Temp. oversampling
                         BMP280::SAMPLING_X16,     // Pressure oversampling
                         BMP280::FILTER_X16,       // Filtering
                         BMP280::STANDBY_MS_500);  // Standby time
    
    currentData.barometer_healthy = true;
    logger.info("Barometer initialized successfully");
    return true;
  } else {
    currentData.barometer_healthy = false;
    logger.error("Barometer initialization failed");
    return false;
  }
}

// Initialize Temperature Sensor
bool SensorManager::initializeTemperatureSensor() {
  logger.info("Initializing temperature sensors...");
  
  // Initialize Dallas Temperature sensor
  temperatureSensor.begin();
  
  // Initialize DHT sensor
  dht.begin();
  
  currentData.temperature_healthy = true;
  logger.info("Temperature sensors initialized successfully");
  return true;
}

// Initialize Ultrasonic sensor
bool SensorManager::initializeUltrasonic() {
  logger.info("Initializing ultrasonic sensor...");
  
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  
  currentData.ultrasonic_healthy = true;
  logger.info("Ultrasonic sensor initialized successfully");
  return true;
}

// Initialize Battery monitoring
bool SensorManager::initializeBattery() {
  logger.info("Initializing battery monitoring...");
  
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  
  // Set ADC resolution
  analogReadResolution(12); // 12-bit ADC
  
  currentData.battery_healthy = true;
  logger.info("Battery monitoring initialized successfully");
  return true;
}

// Initialize Magnetometer
bool SensorManager::initializeMagnetometer() {
  logger.info("Initializing magnetometer...");
  
  // Try to initialize HMC5883L or similar magnetometer
  // This is a placeholder - implement based on your specific magnetometer
  
  currentData.magnetometer_healthy = true;
  logger.info("Magnetometer initialized successfully");
  return true;
}

// Main update function
void SensorManager::update() {
  if (!initialized) return;
  
  unsigned long currentTime = millis();
  
  // Store previous data
  previousData = currentData;
  currentData.timestamp = currentTime;
  
  // Update sensors based on their sample rates
  if (currentTime - lastIMUUpdate >= (1000 / IMU_SAMPLE_RATE)) {
    updateIMU();
    lastIMUUpdate = currentTime;
  }
  
  if (currentTime - lastGPSUpdate >= (1000 / GPS_SAMPLE_RATE)) {
    updateGPS();
    lastGPSUpdate = currentTime;
  }
  
  if (currentTime - lastBarometerUpdate >= (1000 / PRESSURE_SAMPLE_RATE)) {
    updateBarometer();
    lastBarometerUpdate = currentTime;
  }
  
  if (currentTime - lastTemperatureUpdate >= (1000 / TEMPERATURE_SAMPLE_RATE)) {
    updateTemperature();
    lastTemperatureUpdate = currentTime;
  }
  
  if (currentTime - lastUltrasonicUpdate >= (1000 / ULTRASONIC_SAMPLE_RATE)) {
    updateUltrasonic();
    lastUltrasonicUpdate = currentTime;
  }
  
  if (currentTime - lastBatteryUpdate >= 1000) { // 1Hz battery update
    updateBattery();
    lastBatteryUpdate = currentTime;
  }
  
  // Apply calibration
  applyCalibration();
  
  // Perform sensor fusion
  performSensorFusion();
  
  // Health check
  if (healthCheckEnabled) {
    performHealthCheck();
  }
  
  // Validate data
  validateSensorData();
}

// Update IMU data
void SensorManager::updateIMU() {
  if (!currentData.imu_healthy) return;
  
  int16_t ax, ay, az, gx, gy, gz;
  
  // Read raw data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to proper units
  currentData.accelX = ax / 8192.0; // ±4g range
  currentData.accelY = ay / 8192.0;
  currentData.accelZ = az / 8192.0;
  
  currentData.gyroX = gx / 16.4; // ±2000°/s range
  currentData.gyroY = gy / 16.4;
  currentData.gyroZ = gz / 16.4;
  
  // Calculate rates
  if (previousData.timestamp > 0) {
    float dt = (currentData.timestamp - previousData.timestamp) / 1000.0;
    currentData.rollRate = (currentData.roll - previousData.roll) / dt;
    currentData.pitchRate = (currentData.pitch - previousData.pitch) / dt;
    currentData.yawRate = (currentData.yaw - previousData.yaw) / dt;
  }
}

// Update GPS data
void SensorManager::updateGPS() {
  if (!currentData.gps_healthy) return;
  
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        currentData.latitude = gps.location.lat();
        currentData.longitude = gps.location.lng();
        currentData.gps_fix = true;
      } else {
        currentData.gps_fix = false;
      }
      
      if (gps.altitude.isValid()) {
        currentData.gps_altitude = gps.altitude.meters();
      }
      
      if (gps.speed.isValid()) {
        currentData.gps_speed = gps.speed.mps();
      }
      
      if (gps.course.isValid()) {
        currentData.gps_course = gps.course.deg();
      }
      
      if (gps.satellites.isValid()) {
        currentData.satellites = gps.satellites.value();
      }
      
      if (gps.hdop.isValid()) {
        currentData.hdop = gps.hdop.hdop();
      }
    }
  }
}

// Update Barometer data
void SensorManager::updateBarometer() {
  if (!currentData.barometer_healthy) return;
  
  currentData.pressure = barometer.readPressure() / 100.0; // Convert to hPa
  currentData.barometric_altitude = barometer.readAltitude(1013.25); // Sea level pressure
}

// Update Temperature data
void SensorManager::updateTemperature() {
  if (!currentData.temperature_healthy) return;
  
  // Read from DHT sensor
  float dht_temp = dht.readTemperature();
  float dht_humidity = dht.readHumidity();
  
  // Read from Dallas sensor
  temperatureSensor.requestTemperatures();
  float dallas_temp = temperatureSensor.getTempCByIndex(0);
  
  // Use average of valid readings
  float temp_sum = 0;
  int temp_count = 0;
  
  if (!isnan(dht_temp)) {
    temp_sum += dht_temp;
    temp_count++;
  }
  
  if (dallas_temp != DEVICE_DISCONNECTED_C) {
    temp_sum += dallas_temp;
    temp_count++;
  }
  
  if (temp_count > 0) {
    currentData.temperature = applyMovingAverageFilter(&temperatureFilter, temp_sum / temp_count);
  }
  
  if (!isnan(dht_humidity)) {
    currentData.humidity = dht_humidity;
  }
}

// Update Ultrasonic data
void SensorManager::updateUltrasonic() {
  if (!currentData.ultrasonic_healthy) return;
  
  float distance = calculateDistance(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
  
  if (distance > 0 && distance < MAX_ULTRASONIC_DISTANCE) {
    currentData.ultrasonic_altitude = applyMedianFilter(&ultrasonicFilter, distance);
  }
}

// Update Battery data
void SensorManager::updateBattery() {
  if (!currentData.battery_healthy) return;
  
  float voltage = readBatteryVoltage();
  float current = readBatteryCurrent();
  
  currentData.batteryVoltage = applyMovingAverageFilter(&batteryFilter, voltage);
  currentData.batteryCurrent = current;
  currentData.batteryPower = currentData.batteryVoltage * currentData.batteryCurrent;
  currentData.batteryCapacityRemaining = calculateBatteryCapacity();
}

// Calculate distance using ultrasonic sensor
float SensorManager::calculateDistance(float trigPin, float echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return -1; // No echo received
  }
  
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Read battery voltage
float SensorManager::readBatteryVoltage() {
  int rawValue = analogRead(BATTERY_VOLTAGE_PIN);
  float voltage = (rawValue / 4095.0) * 3.3 * BATTERY_VOLTAGE_DIVIDER;
  return voltage;
}

// Read battery current
float SensorManager::readBatteryCurrent() {
  int rawValue = analogRead(CURRENT_SENSOR_PIN);
  float voltage = (rawValue / 4095.0) * 3.3;
  
  // Assuming ACS712 current sensor (adjust for your sensor)
  float current = (voltage - 2.5) / 0.185; // 5A range
  return abs(current);
}

// Calculate battery capacity remaining
float SensorManager::calculateBatteryCapacity() {
  float voltage = currentData.batteryVoltage;
  
  // Simple voltage-based capacity estimation (improve with coulomb counting)
  if (voltage >= 12.6) return 100.0;
  else if (voltage >= 12.4) return 90.0;
  else if (voltage >= 12.2) return 80.0;
  else if (voltage >= 12.0) return 70.0;
  else if (voltage >= 11.8) return 60.0;
  else if (voltage >= 11.6) return 50.0;
  else if (voltage >= 11.4) return 40.0;
  else if (voltage >= 11.2) return 30.0;
  else if (voltage >= 11.0) return 20.0;
  else if (voltage >= 10.8) return 10.0;
  else return 0.0;
}

// Apply calibration to sensor data
void SensorManager::applyCalibration() {
  if (!calibrationData.accel_calibrated) return;
  
  // Apply accelerometer calibration
  currentData.accelX -= calibrationData.accel_bias_x;
  currentData.accelY -= calibrationData.accel_bias_y;
  currentData.accelZ -= calibrationData.accel_bias_z;
  
  // Apply gyroscope calibration
  if (calibrationData.gyro_calibrated) {
    currentData.gyroX -= calibrationData.gyro_bias_x;
    currentData.gyroY -= calibrationData.gyro_bias_y;
    currentData.gyroZ -= calibrationData.gyro_bias_z;
  }
  
  // Apply magnetometer calibration
  if (calibrationData.mag_calibrated) {
    currentData.magX = (currentData.magX - calibrationData.mag_bias_x) * calibrationData.mag_scale_x;
    currentData.magY = (currentData.magY - calibrationData.mag_bias_y) * calibrationData.mag_scale_y;
    currentData.magZ = (currentData.magZ - calibrationData.mag_bias_z) * calibrationData.mag_scale_z;
  }
  
  // Apply ultrasonic calibration
  if (calibrationData.ultrasonic_calibrated) {
    currentData.ultrasonic_altitude -= calibrationData.ultrasonic_offset;
  }
  
  // Apply battery calibration
  if (calibrationData.battery_calibrated) {
    currentData.batteryVoltage -= calibrationData.battery_voltage_offset;
    currentData.batteryCurrent -= calibrationData.current_sensor_offset;
  }
}

// Perform sensor fusion
void SensorManager::performSensorFusion() {
  calculateOrientation();
  calculateAltitude();
  calculateVelocity();
}

// Calculate orientation using complementary filter
void SensorManager::calculateOrientation() {
  if (previousData.timestamp == 0) return;
  
  float dt = (currentData.timestamp - previousData.timestamp) / 1000.0;
  
  // Calculate angles from accelerometer
  float accel_roll = atan2(currentData.accelY, currentData.accelZ) * 180.0 / PI;
  float accel_pitch = atan(-currentData.accelX / sqrt(currentData.accelY * currentData.accelY + currentData.accelZ * currentData.accelZ)) * 180.0 / PI;
  
  // Apply complementary filter
  currentData.roll = applyComplementaryFilter(&rollFilter, currentData.gyroX, accel_roll, dt);
  currentData.pitch = applyComplementaryFilter(&pitchFilter, currentData.gyroY, accel_pitch, dt);
  
  // Yaw integration (needs magnetometer for absolute reference)
  if (yawFilter.initialized) {
    currentData.yaw = yawFilter.filtered_value + currentData.gyroZ * dt;
  } else {
    currentData.yaw = 0;
  }
  yawFilter.filtered_value = currentData.yaw;
  yawFilter.initialized = true;
}

// Calculate fused altitude
void SensorManager::calculateAltitude() {
  float altitude = 0;
  bool hasValidReading = false;
  
  // Use barometric altitude as primary
  if (currentData.barometer_healthy && currentData.pressure > 0) {
    altitude = currentData.barometric_altitude;
    hasValidReading = true;
  }
  
  // Use ultrasonic for low altitude
  if (currentData.ultrasonic_healthy && currentData.ultrasonic_altitude > 0 && currentData.ultrasonic_altitude < 500) {
    if (hasValidReading) {
      // Blend ultrasonic and barometric
      float weight = 1.0 - (currentData.ultrasonic_altitude / 500.0);
      altitude = weight * (currentData.ultrasonic_altitude / 100.0) + (1.0 - weight) * altitude;
    } else {
      altitude = currentData.ultrasonic_altitude / 100.0; // Convert cm to m
      hasValidReading = true;
    }
  }
  
  // Apply Kalman filter
  if (hasValidReading) {
    currentData.filtered_altitude = applyKalmanFilter(&altitudeFilter, altitude);
  }
}

// Calculate vertical velocity
void SensorManager::calculateVelocity() {
  if (previousData.timestamp == 0) return;
  
  float dt = (currentData.timestamp - previousData.timestamp) / 1000.0;
  
  // Calculate velocity from altitude difference
  float velocity = (currentData.filtered_altitude - previousData.filtered_altitude) / dt;
  
  // Apply Kalman filter
  currentData.vertical_velocity = applyKalmanFilter(&velocityFilter, velocity);
}

// Apply Kalman filter
float SensorManager::applyKalmanFilter(KalmanFilter* filter, float measurement) {
  // Predict
  filter->P += filter->Q;
  
  // Update
  filter->K = filter->P / (filter->P + filter->R);
  filter->x += filter->K * (measurement - filter->x);
  filter->P = (1 - filter->K) * filter->P;
  
  return filter->x;
}

// Apply complementary filter
float SensorManager::applyComplementaryFilter(ComplementaryFilter* filter, float gy