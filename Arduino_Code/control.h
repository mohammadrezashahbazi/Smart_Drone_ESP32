#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// پین‌های موتورها
#define MOTOR_1_PIN 6
#define MOTOR_2_PIN 9
#define MOTOR_3_PIN 10
#define MOTOR_4_PIN 11

// محدودیت‌های PWM
#define MIN_PWM 1000
#define MAX_PWM 2000
#define IDLE_PWM 1500

// ثابت‌های کنترل PID
#define KP_ROLL 2.0
#define KI_ROLL 0.1
#define KD_ROLL 0.5

#define KP_PITCH 2.0
#define KI_PITCH 0.1
#define KD_PITCH 0.5

#define KP_YAW 3.0
#define KI_YAW 0.2
#define KD_YAW 0.8

#define KP_ALTITUDE 1.5
#define KI_ALTITUDE 0.05
#define KD_ALTITUDE 0.3

// محدودیت‌های کنترل
#define MAX_ANGLE 30.0        // حداکثر زاویه در درجه
#define MAX_CLIMB_RATE 2.0    // حداکثر سرعت صعود m/s
#define MAX_OUTPUT 500        // حداکثر خروجی PID

// حالت‌های پرواز
enum FlightMode {
  DISARMED,
  ARMED,
  STABILIZE,
  ALTITUDE_HOLD,
  POSITION_HOLD,
  AUTO,
  RETURN_TO_HOME,
  EMERGENCY
};

// ساختار کنترل PID
struct PIDController {
  float kp, ki, kd;
  float previousError;
  float integral;
  float output;
  unsigned long lastTime;
  
  PIDController(float p = 0, float i = 0, float d = 0) : 
    kp(p), ki(i), kd(d), previousError(0), integral(0), output(0), lastTime(0) {}
};

// ساختار ورودی کنترل
struct ControlInput {
  float roll;      // -1 تا 1
  float pitch;     // -1 تا 1
  float yaw;       // -1 تا 1
  float throttle;  // 0 تا 1
  bool armSwitch;
  FlightMode mode;
  
  ControlInput() : roll(0), pitch(0), yaw(0), throttle(0), armSwitch(false), mode(DISARMED) {}
};

// ساختار خروجی موتور
struct MotorOutput {
  int motor1;
  int motor2;
  int motor3;
  int motor4;
  
  MotorOutput() : motor1(IDLE_PWM), motor2(IDLE_PWM), motor3(IDLE_PWM), motor4(IDLE_PWM) {}
};

// ساختار تنظیمات هدف
struct Setpoint {
  float roll;
  float pitch;
  float yaw;
  float altitude;
  float latitude;
  float longitude;
  
  Setpoint() : roll(0), pitch(0), yaw(0), altitude(0), latitude(0), longitude(0) {}
};

// کلاس کنترلر اصلی
class FlightController {
private:
  // PID کنترلرها
  PIDController rollPID;
  PIDController pitchPID;
  PIDController yawPID;
  PIDController altitudePID;
  
  // موتورها
  Servo motor1, motor2, motor3, motor4;
  
  // متغیرهای وضعیت
  FlightMode currentMode;
  bool isArmed;
  bool motorsInitialized;
  
  // تنظیمات
  Setpoint targetSetpoint;
  ControlInput lastInput;
  MotorOutput lastOutput;
  
  // تایمرها
  unsigned long lastControlUpdate;
  unsigned long armTime;
  
  // متدهای خصوصی
  float calculatePID(PIDController& pid, float error, float dt);
  void limitOutput(float& output, float maxOutput);
  void mixerQuadX(float roll, float pitch, float yaw, float throttle, MotorOutput& output);
  bool safetyChecks();
  void emergencyStop();
  
public:
  FlightController();
  
  // متدهای اصلی
  bool initialize();
  void update(const ControlInput& input, float currentRoll, float currentPitch, 
              float currentYaw, float currentAltitude);
  void setMode(FlightMode mode);
  void arm();
  void disarm();
  
  // متدهای کنترل
  void stabilizeMode(const ControlInput& input, float currentRoll, float currentPitch, float currentYaw);
  void altitudeHoldMode(const ControlInput& input, float currentRoll, float currentPitch, 
                       float currentYaw, float currentAltitude);
  void positionHoldMode(const ControlInput& input, float currentRoll, float currentPitch,
                       float currentYaw, float currentAltitude, float currentLat, float currentLon);
  void returnToHomeMode(float currentRoll, float currentPitch, float currentYaw, 
                       float currentAltitude, float currentLat, float currentLon);
  
  // متدهای تنظیمات
  void setSetpoint(const Setpoint& setpoint);
  void setPIDGains(char axis, float kp, float ki, float kd);
  void calibrateMotors();
  
  // متدهای دسترسی
  FlightMode getMode() const { return currentMode; }
  bool getArmedStatus() const { return isArmed; }
  MotorOutput getMotorOutput() const { return lastOutput; }
  
  // متدهای اضطراری
  void emergencyLand();
  void killSwitch();
  
  // متدهای تست
  void testMotors();
  void setMotorSpeed(int motorNum, int speed);
};

// توابع کمکی سراسری
void initializeFlightController();
void updateFlightController();
void setFlightMode(FlightMode mode);
void armDrone();
void disarmDrone();
bool isFlightControllerReady();

// متغیرهای سراسری
extern FlightController flightController;
extern ControlInput controlInput;
extern MotorOutput motorOutput;
extern bool emergencyStopActive;

// توابع ریاضی کمکی
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax);
float constrainFloat(float value, float min, float max);
float lowPassFilter(float input, float previousOutput, float alpha);
float deadBand(float input, float threshold);

// توابع کالیبراسیون
void calibrateESCs();
void setMotorLimits(int minPWM, int maxPWM);

// ثابت‌های فیلتر
#define GYRO_FILTER_ALPHA 0.8
#define ACCEL_FILTER_ALPHA 0.2
#define CONTROL_LOOP_RATE 100  // Hz

#endif // CONTROL_H
