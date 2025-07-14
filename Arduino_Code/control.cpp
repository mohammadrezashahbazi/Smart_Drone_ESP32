#include "control.h"
#include "sensors.h"

// متغیرهای سراسری
FlightController flightController;
ControlInput controlInput;
MotorOutput motorOutput;
bool emergencyStopActive = false;

// سازنده کلاس FlightController
FlightController::FlightController() :
  rollPID(KP_ROLL, KI_ROLL, KD_ROLL),
  pitchPID(KP_PITCH, KI_PITCH, KD_PITCH),
  yawPID(KP_YAW, KI_YAW, KD_YAW),
  altitudePID(KP_ALTITUDE, KI_ALTITUDE, KD_ALTITUDE),
  currentMode(DISARMED),
  isArmed(false),
  motorsInitialized(false),
  lastControlUpdate(0),
  armTime(0)
{
}

// راه‌اندازی کنترلر پرواز
bool FlightController::initialize() {
  // راه‌اندازی موتورها
  motor1.attach(MOTOR_1_PIN);
  motor2.attach(MOTOR_2_PIN);
  motor3.attach(MOTOR_3_PIN);
  motor4.attach(MOTOR_4_PIN);
  
  // تنظیم موتورها روی حالت بی‌حرکت
  motor1.writeMicroseconds(IDLE_PWM);
  motor2.writeMicroseconds(IDLE_PWM);
  motor3.writeMicroseconds(IDLE_PWM);
  motor4.writeMicroseconds(IDLE_PWM);
  
  delay(1000); // انتظار برای راه‌اندازی ESCها
  
  motorsInitialized = true;
  currentMode = DISARMED;
  
  Serial.println("Flight Controller initialized successfully");
  return true;
}

// به‌روزرسانی کنترلر پرواز
void FlightController::update(const ControlInput& input, float currentRoll, float currentPitch, 
                             float currentYaw, float currentAltitude) {
  
  unsigned long currentTime = millis();
  
  // بررسی نرخ به‌روزرسانی
  if (currentTime - lastControlUpdate < (1000 / CONTROL_LOOP_RATE)) {
    return;
  }
  
  lastControlUpdate = currentTime;
  lastInput = input;
  
  // بررسی‌های ایمنی
  if (!safetyChecks()) {
    emergencyStop();
    return;
  }
  
  // بررسی درخواست تسلیح/خلع سلاح
  if (input.armSwitch && !isArmed && currentMode == DISARMED) {
    arm();
  } else if (!input.armSwitch && isArmed) {
    disarm();
  }
  
  // اجرای کنترل بر اساس حالت پرواز
  if (isArmed) {
    switch (currentMode) {
      case STABILIZE:
        stabilizeMode(input, currentRoll, currentPitch, currentYaw);
        break;
        
      case ALTITUDE_HOLD:
        altitudeHoldMode(input, currentRoll, currentPitch, currentYaw, currentAltitude);
        break;
        
      case POSITION_HOLD:
        positionHoldMode(input, currentRoll, currentPitch, currentYaw, currentAltitude, 
                        latitude, longitude);
        break;
        
      case RETURN_TO_HOME:
        returnToHomeMode(currentRoll, currentPitch, currentYaw, currentAltitude, 
                        latitude, longitude);
        break;
        
      case EMERGENCY:
        emergencyLand();
        break;
        
      default:
        disarm();
        break;
    }
  } else {
    // خلع سلاح - موتورها روی حالت بی‌حرکت
    lastOutput.motor1 = IDLE_PWM;
    lastOutput.motor2 = IDLE_PWM;
    lastOutput.motor3 = IDLE_PWM;
    lastOutput.motor4 = IDLE_PWM;
  }
  
  // ارسال سیگنال به موتورها
  motor1.writeMicroseconds(lastOutput.motor1);
  motor2.writeMicroseconds(lastOutput.motor2);
  motor3.writeMicroseconds(lastOutput.motor3);
  motor4.writeMicroseconds(lastOutput.motor4);
}

// محاسبه PID
float FlightController::calculatePID(PIDController& pid, float error, float dt) {
  // محاسبه جمله‌های PID
  float proportional = pid.kp * error;
  
  pid.integral += error * dt;
  pid.integral = constrainFloat(pid.integral, -MAX_OUTPUT/pid.ki, MAX_OUTPUT/pid.ki);
  float integral = pid.ki * pid.integral;
  
  float derivative = pid.kd * (error - pid.previousError) / dt;
  pid.previousError = error;
  
  // محاسبه خروجی نهایی
  pid.output = proportional + integral + derivative;
  limitOutput(pid.output, MAX_OUTPUT);
  
  return pid.output;
}

// محدود کردن خروجی
void FlightController::limitOutput(float& output, float maxOutput) {
  output = constrainFloat(output, -maxOutput, maxOutput);
}

// میکسر کوادکوپتر X
void FlightController::mixerQuadX(float roll, float pitch, float yaw, float throttle, MotorOutput& output) {
  // محاسبه خروجی هر موتور برای کوادکوپتر X
  float motor1_output = throttle - pitch + roll - yaw;  // جلو راست
  float motor2_output = throttle - pitch - roll + yaw;  // جلو چپ
  float motor3_output = throttle + pitch - roll - yaw;  // عقب راست
  float motor4_output = throttle + pitch + roll + yaw;  // عقب چپ
  
  // تبدیل به PWM
  output.motor1 = constrain(IDLE_PWM + motor1_output, MIN_PWM, MAX_PWM);
  output.motor2 = constrain(IDLE_PWM + motor2_output, MIN_PWM, MAX_PWM);
  output.motor3 = constrain(IDLE_PWM + motor3_output, MIN_PWM, MAX_PWM);
  output.motor4 = constrain(IDLE_PWM + motor4_output, MIN_PWM, MAX_PWM);
}

// بررسی‌های ایمنی
bool FlightController::safetyChecks() {
  // بررسی وضعیت باتری
  if (batteryVoltage < 10.5) { // حداقل ولتاژ برای LiPo 3S
    Serial.println("WARNING: Low battery voltage!");
    return false;
  }
  
  // بررسی وضعیت سنسورها
  if (!getIMUHealth() || !getBatteryHealth()) {
    Serial.println("WARNING: Sensor failure!");
    return false;
  }
  
  // بررسی توقف اضطراری
  if (emergencyStopActive) {
    Serial.println("WARNING: Emergency stop active!");
    return false;
  }
  
  return true;
}

// توقف اضطراری
void FlightController::emergencyStop() {
  isArmed = false;
  currentMode = EMERGENCY;
  
  lastOutput.motor1 = IDLE_PWM;
  lastOutput.motor2 = IDLE_PWM;
  lastOutput.motor3 = IDLE_PWM;
  lastOutput.motor4 = IDLE_PWM;
  
  motor1.writeMicroseconds(IDLE_PWM);
  motor2.writeMicroseconds(IDLE_PWM);
  motor3.writeMicroseconds(IDLE_PWM);
  motor4.writeMicroseconds(IDLE_PWM);
  
  Serial.println("EMERGENCY STOP ACTIVATED!");
}

// حالت تعادل
void FlightController::stabilizeMode(const ControlInput& input, float currentRoll, 
                                   float currentPitch, float currentYaw) {
  
  float dt = 0.01; // 100Hz control loop
  
  // محاسبه خطا
  float rollError = (input.roll * MAX_ANGLE) - currentRoll;
  float pitchError = (input.pitch * MAX_ANGLE) - currentPitch;
  float yawError = input.yaw * 180.0 - currentYaw; // درجه در ثانیه
  
  // محاسبه PID
  float rollOutput = calculatePID(rollPID, rollError, dt);
  float pitchOutput = calculatePID(pitchPID, pitchError, dt);
  float yawOutput = calculatePID(yawPID, yawError, dt);
  
  // تبدیل throttle به PWM
  float throttleOutput = input.throttle * 400; // 0-400 PWM range
  
  // میکسر موتور
  mixerQuadX(rollOutput, pitchOutput, yawOutput, throttleOutput, lastOutput);
}

// حالت نگهداری ارتفاع
void FlightController::altitudeHoldMode(const ControlInput& input, float currentRoll, 
                                       float currentPitch, float currentYaw, float currentAltitude) {
  
  float dt = 0.01;
  
  // کنترل Roll و Pitch مثل حالت تعادل
  float rollError = (input.roll * MAX_ANGLE) - currentRoll;
  float pitchError = (input.pitch * MAX_ANGLE) - currentPitch;
  float yawError = input.yaw * 180.0 - currentYaw;
  
  float rollOutput = calculatePID(rollPID, rollError, dt);
  float pitchOutput = calculatePID(pitchPID, pitchError, dt);
  float yawOutput = calculatePID(yawPID, yawError, dt);
  
  // کنترل ارتفاع
  if (input.throttle > 0.6) {
    targetSetpoint.altitude += MAX_CLIMB_RATE * dt;
  } else if (input.throttle < 0.4) {
    targetSetpoint.altitude -= MAX_CLIMB_RATE * dt;
  }
  
  float altitudeError = targetSetpoint.altitude - currentAltitude;
  float altitudeOutput = calculatePID(altitudePID, altitudeError, dt);
  
  // ترکیب با throttle پایه
  float throttleOutput = 200 + altitudeOutput; // throttle پایه + تصحیح ارتفاع
  
  mixerQuadX(rollOutput, pitchOutput, yawOutput, throttleOutput, lastOutput);
}

// حالت نگهداری موقعیت
void FlightController::positionHoldMode(const ControlInput& input, float currentRoll, 
                                       float currentPitch, float currentYaw, float currentAltitude,
                                       float currentLat, float currentLon) {
  
  // کنترل ارتفاع
  altitudeHoldMode(input, currentRoll, currentPitch, currentYaw, currentAltitude);
  
  // کنترل موقعیت (ساده)
  if (abs(input.roll) < 0.1 && abs(input.pitch) < 0.1) {
    // حالت نگهداری - هدف‌گذاری موقعیت فعلی
    targetSetpoint.latitude = currentLat;
    targetSetpoint.longitude = currentLon;
  }
}

// حالت بازگشت به خانه
void FlightController::returnToHomeMode(float currentRoll, float currentPitch, float currentYaw, 
                                       float currentAltitude, float currentLat, float currentLon) {
  
  // محاسبه فاصله و جهت تا خانه
  float distance = calculateDistance(currentLat, currentLon, 
                                   targetSetpoint.latitude, targetSetpoint.longitude);
  
  if (distance < 2.0) { // اگر نزدیک خانه باشیم
    emergencyLand();
    return;
  }
  
  float bearing = calculateBearing(currentLat, currentLon, 
                                 targetSetpoint.latitude, targetSetpoint.longitude);
  
  // ایجاد ورودی کنترل برای حرکت به سمت خانه
  ControlInput homeInput;
  
  float yawError = bearing - currentYaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;
  
  homeInput.yaw = constrainFloat(yawError / 90.0, -1.0, 1.0);
  homeInput.pitch = constrainFloat(distance / 10.0, -0.5, 0.5);
  homeInput.throttle = 0.5; // throttle متوسط
  
  stabilizeMode(homeInput, currentRoll, currentPitch, currentYaw);
}

// تسلیح
void FlightController::arm() {
  if (!motorsInitialized) return;
  
  // بررسی شرایط تسلیح
  if (abs(roll) > 5.0 || abs(pitch) > 5.0) {
    Serial.println("Cannot arm: Drone not level");
    return;
  }
  
  if (batteryVoltage < 11.0) {
    Serial.println("Cannot arm: Battery too low");
    return;
  }
  
  isArmed = true;
  currentMode = STABILIZE;
  armTime = millis();
  
  // صفر کردن PID
  rollPID.integral = 0;
  pitchPID.integral = 0;
  yawPID.integral = 0;
  altitudePID.integral = 0;
  
  Serial.println("ARMED - Ready for flight");
}

// خلع سلاح
void FlightController::disarm() {
  isArmed = false;
  currentMode = DISARMED;
  
  lastOutput.motor1 = IDLE_PWM;
  lastOutput.motor2 = IDLE_PWM;
  lastOutput.motor3 = IDLE_PWM;
  lastOutput.motor4 = IDLE_PWM;
  
  Serial.println("DISARMED");
}

// تنظیم حالت پرواز
void FlightController::setMode(FlightMode mode) {
  if (!isArmed && mode != DISARMED) {
    Serial.println("Cannot change mode: Drone not armed");
    return;
  }
  
  currentMode = mode;
  
  // تنظیم setpoint برای حالت جدید
  if (mode == ALTITUDE_HOLD) {
    targetSetpoint.altitude = altitude;
  } else if (mode == POSITION_HOLD) {
    targetSetpoint.latitude = latitude;
    targetSetpoint.longitude = longitude;
    targetSetpoint.altitude = altitude;
  }
  
  Serial.print("Mode changed to: ");
  Serial.println(mode);
}

// فرود اضطراری
void FlightController::emergencyLand() {
  // کاهش تدریجی قدرت موتورها
  static float landingThrottle = 200;
  
  if (altitude > 0.5) {
    landingThrottle -= 2; // کاهش تدریجی
  } else {
    landingThrottle = 0; // خاموش کردن موتورها
    disarm();
  }
  
  lastOutput.motor1 = IDLE_PWM + landingThrottle;
  lastOutput.motor2 = IDLE_PWM + landingThrottle;
  lastOutput.motor3 = IDLE_PWM + landingThrottle;
  lastOutput.motor4 = IDLE_PWM + landingThrottle;
}

// کلید کشتن
void FlightController::killSwitch() {
  emergencyStopActive = true;
  emergencyStop();
}

// تست موتورها
void FlightController::testMotors() {
  if (isArmed) {
    Serial.println("Cannot test motors while armed");
    return;
  }
  
  Serial.println("Testing motors...");
  
  // تست هر موتور به مدت 2 ثانیه
  int testSpeed = 1200; // سرعت کم برای تست
  
  for (int i = 1; i <= 4; i++) {
    Serial.print("Testing motor ");
    Serial.println(i);
    
    setMotorSpeed(i, testSpeed);
    delay(2000);
    setMotorSpeed(i, IDLE_PWM);
    delay(500);
  }
  
  Serial.println("Motor test complete");
}

// تنظیم سرعت موتور
void FlightController::setMotorSpeed(int motorNum, int speed) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  
  switch (motorNum) {
    case 1: motor1.writeMicroseconds(speed); break;
    case 2: motor2.writeMicroseconds(speed); break;
    case 3: motor3.writeMicroseconds(speed); break;
    case 4: motor4.writeMicroseconds(speed); break;
  }
}

// تنظیم ضرایب PID
void FlightController::setPIDGains(char axis, float kp, float ki, float kd) {
  switch (axis) {
    case 'R':
    case 'r':
      rollPID.kp = kp;
      rollPID.ki = ki;
      rollPID.kd = kd;
      break;
    case 'P':
    case 'p':
      pitchPID.kp = kp;
      pitchPID.ki = ki;
      pitchPID.kd = kd;
      break;
    case 'Y':
    case 'y':
      yawPID.kp = kp;
      yawPID.ki = ki;
      yawPID.kd = kd;
      break;
    case 'A':
    case 'a':
      altitudePID.kp = kp;
      altitudePID.ki = ki;
      altitudePID.kd = kd;
      break;
  }
}

// توابع کمکی سراسری
void initializeFlightController() {
  flightController.initialize();
}

void updateFlightController() {
  flightController.update(controlInput, roll, pitch, yaw, altitude);
  motorOutput = flightController.getMotorOutput();
}

void setFlightMode(FlightMode mode) {
  flightController.setMode(mode);
}

void armDrone() {
  flightController.arm();
}

void disarmDrone() {
  flightController.disarm();
}

bool isFlightControllerReady() {
  return flightController.getArmedStatus();
}

// توابع ریاضی کمکی
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float constrainFloat(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

float lowPassFilter(float input, float previousOutput, float alpha) {
  return alpha * input + (1.0 - alpha) * previousOutput;
}

float deadBand(float input, float threshold) {
  if (abs(input) < threshold) return 0;
  return input;
}

// کالیبراسیون ESC
void calibrateESCs() {
  Serial.println("Starting ESC calibration...");
  Serial.println("Connect battery NOW!");
  
  // ارسال سیگنال حداکثر
  flightController.setMotorSpeed(1, MAX_PWM);
  flightController.setMotorSpeed(2, MAX_PWM);
  flightController.setMotorSpeed(3, MAX_PWM);
  flightController.setMotorSpeed(4, MAX_PWM);
  
  delay(5000);
  
  // ارسال سیگنال حداقل
  flightController.setMotorSpeed(1, MIN_PWM);
  flightController.setMotorSpeed(2, MIN_PWM);
  flightController.setMotorSpeed(3, MIN_PWM);
  flightController.setMotorSpeed(4, MIN_PWM);
  
  delay(3000);
  
  // بازگشت به حالت بی‌حرکت
  flightController.setMotorSpeed(1, IDLE_PWM);
  flightController.setMotorSpeed(2, IDLE_PWM);
  flightController.setMotorSpeed(3, IDLE_PWM);
  flightController.setMotorSpeed(4, IDLE_PWM);
  
  Serial.println("ESC calibration complete!");
}

void setMotorLimits(int minPWM, int maxPWM) {
  // این تابع برای تنظیم محدودیت‌های PWM استفاده می‌شود
  // در صورت نیاز می‌توان آن را پیاده‌سازی کرد
}
