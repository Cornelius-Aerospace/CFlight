#ifndef MAIN_H
#define MAIN_H
//#define SIM_MODE // comment out for physical use of sensors
// Include libraries
#include "I2Cdev.h"
#ifndef SIM_MODE
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#endif
#include <Wire.h>
#include "WiFi.h"
#include "SPI.h"
#include <time.h>

#include "comms.h"
#include "common.h"
#include "const.h"
#include "settings.h"
#include "flog.h"

#ifdef __cplusplus
extern "C"
{
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

char fmtBuffer[128] = ""; // Initialize the buffer as an empty string

#ifdef AUTO_ARM
unsigned long autoArmCalibratedBy = 0;
bool autoArmed = false;
#endif

const unsigned long idleLedPattern[] = {
    LED_ON_SHORT,
    LED_OFF_SHORT,
    LED_ON_SHORT,
    LED_OFF_LONG,
};

const unsigned long calibrateLedPattern[] = {
    LED_ON_LONG, LED_OFF_SHORT};

const unsigned long armedLedPattern[] = {
    LED_ON_SHORT, LED_OFF_SHORT};

const unsigned long groundLedPattern[] = {
    LED_ON_LONG,
    LED_OFF_MED,
    LED_ON_SHORT,
    LED_OFF_LONG,
};

float *altitudeHistory;
float *speedHistory;
unsigned long logIndex = 0;
unsigned long finalLogIndex = 0;
unsigned long lastHistoryEntry = 0;
bool loggingData = false;

const char *SSID = "CFlight";
const char *psk = "there IS no sp00n";
WiFiServer wifiServer(8080);
WiFiClient ground_station;
uint16_t flight_id = 0;
#ifndef SIM_MODE
MPU6050 accelgyro;   // I2C MPU6050 IMU
Adafruit_BMP280 bmp; // I2C bmp280 barometer for altitude
TinyGPSPlus gps;     // Serial GPS
#endif

bool mpu_state = false;
bool bmp_state = false;

bool flightConfigured = false;

// Command holders from comms.h
extern unsigned long commandSalt;
extern uint8_t commandInt;
extern Command command;
extern uint8_t commandArgLength;
extern uint8_t commandArgCount;
extern byte commandArgBuffer[MAX_ARGS * 4];

extern SystemSettings SysSettings;
extern FlightSettings ActiveFlightSettings;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float pressure, bmpTemperature, altitude, previousAltitude;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long lastSpeedMeasurement = 0;
unsigned long deltaTime = 0;

float deltaAltitude = 0;
float peakAltitude = 0; // The largest altitude seen so far (locked at descent)
float verticalVelocity = 0;
float peakVerticalVelocity = 0; // The largest vertical velocity seen so far (locked at descent)
unsigned long peakVerticalVelocityTime = 0;

bool calibrate_done = false;
const char *StateNames[] = {"IDLE", "CALIBRATE", "ARMED", "ASCENT", "DESCENT", "LANDED", "GROUNDSTATION", "ERROR"};
bool operationStatus = false;
unsigned long errorCount = 0;

bool hasFlown = false;
State state = State::IDLE;
uint8_t stateChanged = 0; // Has the state changed since last tick(); (0 - no, 1 - yes, 2 - state changing this tick (ignore))
String commandCollector = "";

String commandTypeHolder = "";
String commandArgs[MAX_ARGS] = {};

uint16_t launchDetectTicker = 0;
uint16_t descentTicker = 0;
uint16_t landedTicker = 0;

Command activeCommand = Command::NONE;
unsigned long commandData = 0; // Data passed with command

bool drougeFired = false;    // set true after drouge pyro fired
bool mainChuteFired = false; // Set true after dual deploy main chute fired
bool stageChFired = false;   // seccond stage motor has been ignited)

bool masterArm = false; // Set true when entering arm state, false when unarmed, landed or ground station entered

unsigned long lastLedEvent = 0;
uint8_t activeLedPattern = 0;
uint16_t ledPatternStage = 0;

bool buzzerState = false;

float initalPresure = 0.0;                                                            // Presure at launch pad
unsigned long launchEventTimestamp, apoggeeEventTimestamp, landingEventTimestamp = 0; // Time at which the events were detected
unsigned long lastLogEvent = 0;

float speedTicker = 0;
unsigned long gpsSatilliteCount = 0;
double gpsLatitude, gpsLongitude, gpsAltitude = 0;
bool gpsFix = false;
int32_t gpsHdop = 0;
double gpsSpeed = 0.0;

// Logging wrappers
// TODO: log file on flash (in dedicated slot)
template <typename T>
inline void printlog(const T &value)
{
    Serial.print(value);
}

template <typename T>
inline void printlnlog(const T &value)
{
    Serial.println(value);
}

inline void printlnlog()
{
    Serial.println();
}

inline void printlog()
{
    Serial.println();
}
const uint8_t maxStringLength = 255;
char formattedString[maxStringLength];

void printlogf(const char *format, ...)
{
    // Maximum length for the formatted string, adjust as needed

    // Initialize variable argument list
    va_list args;
    va_start(args, format);

    // Format the string using vsnprintf
    uint8_t formattedLength = vsnprintf(formattedString, maxStringLength, format, args);

    // Check if the formatted string fits within the buffer
    if (formattedLength >= 0 && formattedLength < maxStringLength)
    {
        printlog(formattedString);
    }
    else
    {
        // Handle buffer overflow or formatting errors
        Serial.println("Error: Formatted string too long or other formatting error.");
    }
    // Cleanup the variable argument list
    formattedString[0] = '\0';
    va_end(args);
}
// End logging wrappers

TaskHandle_t detectEventsTaskHandle = NULL;
TaskHandle_t logDataTaskHandle = NULL;
TaskHandle_t pollSensorsTaskHandle = NULL;
TaskHandle_t updateOutputsTaskHandle = NULL;
TaskHandle_t systemReportTaskHandle = NULL;

// Function prototypes
void setup();
void loop();
void allocateHistoryMemory();
void updateAccelOffset(SystemSettings settings);
void updateSystemSettings(SystemSettings newSystemSettings);
void initSensors();
void pollGps();
uint16_t readFlightId();
bool saveMetaData();
void registerTasks();
void pollSensors(void *param);
void detectEvents(void *param);
void logData(void *param);
void pollImu();
void pollBmp();
bool firePyroCH(uint8_t channel);
void eventEntry(char *s);
void stateChange(State newState);
void systemCheck();
void humanLogTimestamp(unsigned long timestamp);
String formatTimestamp(unsigned long timestamp);
void report();
float cpuTemp();
void systemReport();
void systemReportTask(void *params);
void updateOutputs(void *params);
void minimalLog();
bool commandPacketCallback(uint8_t *responsePacketBuffer, uint8_t *responsePacketLength, uint8_t *args, uint8_t argsCount, uint8_t argsArrayLength, Command cmd, unsigned long time, unsigned long salt);

// Flight log
void listSlotsToSerial();
void dumpSlotToSerial(uint8_t slotID, uint8_t chunkSize = 256, bool fastRead = false);
void writeSerialToSlot(uint8_t slotID, const char terminator = '\n', bool errorChecking = true);
#endif
