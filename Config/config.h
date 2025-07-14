#ifndef CONFIG_H
#define CONFIG_H

// ================================
// تنظیمات کلی سیستم
// ================================

// نسخه فریم‌ور
#define FIRMWARE_VERSION "1.0.0"
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// تنظیمات سریال
#define SERIAL_BAUD_RATE 115200
#define DEBUG_ENABLED true

// ================================
// تنظیمات هاردور
// ================================

// نوع کنترلر
#define CONTROLLER_TYPE "ESP32"
#define DRONE_TYPE "QUADCOPTER_X"

// فرکانس پردازنده
#define CPU_FREQUENCY 240 // MHz

// تنظیمات PWM
#define PWM_FREQUENCY 50    // Hz
#define PWM_RESOLUTION 16   // bits

// ================================
// تنظیمات موتور و ESC
// ================================

// محدودیت‌های PWM موتور
#define MOTOR_MIN_PWM 1000
#define MOTOR_MAX_PWM 2000
#define MOTOR_IDLE_PWM 1000
#define MOTOR_ARM_PWM 1100

// پین‌های موتور
#define MOTOR_1_PIN 12  // جلو راست
#define MOTOR_2_PIN 13  // جلو چپ
#define MOTOR_3_PIN 14  // عقب راست
#define MOTOR_4_PIN 15  // عقب چپ

// تنظیمات ESC
#define ESC_CALIBRATION_ENABLED true
#define ESC_ARM_DELAY 3000 // ms

// ================================
// تنظیمات سنسور
// ================================

// IMU (MPU6050)
#define IMU_ADDRESS 0x68
#define IMU_CALIBRATION_SAMPLES 2000
#define IMU_FILTER_ALPHA 0.98

// GPS (NEO-8M)
#define GPS_BAUD_RATE 9600
#define GPS_UPDATE_RATE 10 // Hz

// بارومتر (BMP280)
#define BAROMETER_ADDRESS 0x77
#define BAROMETER_FILTER_ALPHA 0.95

// کامپس (HMC5883L)
#define COMPASS_ADDRESS 0x1E
#define COMPASS_DECLINATION 4.5 // درجه (بسته به موقعیت جغرافیایی)

// ================================
// تنظیمات کنترل پرواز
// ================================

// نرخ حلقه کنترل
#define CONTROL_LOOP_RATE 100 // Hz
#define SENSOR_UPDATE_RATE 100 // Hz

// محدودیت‌های زاویه
#define MAX_ROLL_ANGLE 30.0    // درجه
#define MAX_PITCH_ANGLE 30.0   // درجه
#define MAX_YAW_RATE 180.0     // درجه/ثانیه

// محدودیت‌های سرعت
#define MAX_CLIMB_RATE 5.0     // متر/ثانیه
#define MAX_DESCENT_RATE 3.0   // متر/ثانیه
#define MAX_HORIZONTAL_SPEED 10.0 // متر/ثانیه

// تنظیمات PID - Roll
#define PID_ROLL_KP 2.0
#define PID_ROLL_KI 0.1
#define PID_ROLL_KD 0.05
#define PID_ROLL_MAX_OUTPUT 400

// تنظیمات PID - Pitch
#define PID_PITCH_KP 2.0
#define PID_PITCH_KI 0.1
#define PID_PITCH_KD 0.05
#define PID_PITCH_MAX_OUTPUT 400

// تنظیمات PID - Yaw
#define PID_YAW_KP 3.0
#define PID_YAW_KI 0.2
#define PID_YAW_KD 0.1
#define PID_YAW_MAX_OUTPUT 400

// تنظیمات PID - Altitude
#define PID_ALTITUDE_KP 1.5
#define PID_ALTITUDE_KI 0.05
#define PID_ALTITUDE_KD 0.8
#define PID_ALTITUDE_MAX_OUTPUT 300

// ================================
// تنظیمات ایمنی
// ================================

// باتری
#define BATTERY_MIN_VOLTAGE 10.5   // ولت (برای LiPo 3S)
#define BATTERY_CRITICAL_VOLTAGE 10.0 // ولت
#define BATTERY_MAX_VOLTAGE 12.6   // ولت

// محدودیت‌های ایمنی
#define MAX_FLIGHT_TIME 900000     // 15 دقیقه (ms)
#define MAX_ALTITUDE 100.0         // متر
#define MIN_ALTITUDE 0.5           // متر

// تایم‌اوت‌ها
#define RC_TIMEOUT 1000           // ms
#define GPS_TIMEOUT 5000          // ms
#define SENSOR_TIMEOUT 500        // ms

// ================================
// تنظیمات ارتباطات
// ================================

// WiFi
#define WIFI_SSID "DroneControl"
#define WIFI_PASSWORD "12345678"
#define WIFI_CHANNEL 1
#define WIFI_MAX_CONNECTIONS 4

// WebServer
#define WEB_SERVER_PORT 80
#define API_RATE_LIMIT 10 // req/sec

// Serial Communication
#define RC_SERIAL_BAUD 9600
#define TELEMETRY_RATE 10 // Hz

// ================================
// تنظیمات حافظه
// ================================

// EEPROM
#define EEPROM_SIZE 1024
#define EEPROM_MAGIC_NUMBER 0xABCD

// آدرس‌های EEPROM
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_PID_ROLL 4
#define EEPROM_ADDR_PID_PITCH 16
#define EEPROM_ADDR_PID_YAW 28
#define EEPROM_ADDR_PID_ALTITUDE 40
#define EEPROM_ADDR_CALIBRATION 52

// ================================
// تنظیمات Debug
// ================================

// سطح Debug
#define DEBUG_LEVEL_ERROR 1
#define DEBUG_LEVEL_WARNING 2
#define DEBUG_LEVEL_INFO 3
#define DEBUG_LEVEL_DEBUG 4

#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_INFO

// ماکروهای Debug
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif

// ================================
// تنظیمات LED و صدا
// ================================

// پین‌های LED
#define LED_STATUS_PIN 2
#define LED_GPS_PIN 4
#define LED_ARMED_PIN 5

// پین buzzer
#define BUZZER_PIN 18

// الگوهای LED
#define LED_PATTERN_BOOT 1
#define LED_PATTERN_ARMED 2
#define LED_PATTERN_DISARMED 3
#define LED_PATTERN_ERROR 4
#define LED_PATTERN_GPS_FIX 5

// ================================
// تنظیمات پیشرفته
// ================================

// فیلتر کالمن
#define KALMAN_FILTER_ENABLED true
#define KALMAN_PROCESS_NOISE 0.01
#define KALMAN_MEASUREMENT_NOISE 0.1

// تنظیمات GPS
#define GPS_MIN_SATELLITES 6
#define GPS_HDOP_THRESHOLD 2.0

// تنظیمات کامپس
#define COMPASS_CALIBRATION_ENABLED true
#define COMPASS_DECLINATION_AUTO true

// تنظیمات مود‌های پرواز
#define FLIGHT_MODE_STABILIZE_ENABLED true
#define FLIGHT_MODE_ALTITUDE_HOLD_ENABLED true
#define FLIGHT_MODE_POSITION_HOLD_ENABLED true
#define FLIGHT_MODE_RTH_ENABLED true
#define FLIGHT_MODE_AUTO_ENABLED false

// ================================
// ماکروهای کمکی
// ================================

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
#define DEGREES_TO_RADIANS(deg) ((deg) * PI / 180.0)
#define RADIANS_TO_DEGREES(rad) ((rad) * 180.0 / PI)

// توابع کمکی
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#define UNUSED(x) ((void)(x))

#endif // CONFIG_H
