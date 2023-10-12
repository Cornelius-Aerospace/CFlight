#ifndef CONST_H
#define CONST_H

#define DEBUG // Comment out to disable debug messages
// #define COMM_TEST

#define VERSION "0.2.0"
#define MASTER_CODE 202910
#define TEMP_ARM_CODE 12345
// // I2C pins (SDA, SCL)
// #define SDA 21
// #define SCL 22

// GPS serial2 pins (RX, TX)
#define GPS_RX 16
#define GPS_TX 17


// LED patterns
#define LED_ON_SHORT 100
#define LED_ON_MED 200
#define LED_ON_LONG 1000

#define LED_OFF_SHORT 100
#define LED_OFF_MED 300
#define LED_OFF_LONG 1000

#define LAUNCH_DETECT_THRESHOLD 1.0 // Meters
#define LAUNCH_DETECT_TICKS 10     // This many ticks for confirmed launch

#define DESCENT_DETECT_THRESHOLD 1.0
#define DESCENT_DETECT_TICKS 10

#define LANDED_DETECT_THRESHOLD_LOW -0.4 // Meters
#define LANDED_DETECT_THRESHOLD_HIGH 0.4 // Meters
#define LANDED_DETECT_TICKS 100           // Number of ticks with subthreshold altitude change for landing event

#define LOG_INTERVAL 1000 // ms between (minimal) data log
#define SPEED_INTERVAL 50 // ms between speed calculations

// #define RAM_LOG_ENABLED
#define HISTORY_SIZE 10000  // max dynamically allocated DRAM (15KB) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/memory-types.html
#define HISTORY_INTERVAL 50 // ms between history updates
#define LOG_SENSOR_COUNT 2  // Altitude, Vertical Velocity

#define CSV_HEADER "time,altitude,z_velocity,airtemp,airpressure"
// #define SD_CARD
#define MAX_ARGS 8
#define SENSOR_POLL_INTERVAL 5 // ms between sensor polls
enum State
{
    IDLE,
    CALIBRATE,
    ARMED,
    ASCENT,
    DESCENT,
    LANDED,
    GROUNDSTATION,
    ERROR
};


enum Command
{
    // Settings related
    ARM, // 0
    UNARM,
    SET_FLIGHT,
    SYSTEM_REPORT,
    SET_BUZZER,
    BATTERY_CHECK,
    SLEEP,
    POWER_DOWN,
    READ_FLIGHT,
    NONE // No command (default)
};

#endif