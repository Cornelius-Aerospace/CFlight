#ifndef MAIN_H
#define MAIN_H
// Include libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>

#define DEBUG // Comment out to disable debug messages
#define VERSION "0.2.0"

// I2C pins (SDA, SCL)
#define SDA 21
#define SCL 22

#define STATUS_LED 12
#define ERROR_LED 13
#define BUZZER_PIN 4

// LED patterns
#define LED_ON_SHORT 250
#define LED_ON_MED 500
#define LED_ON_LONG 1000

#define LED_OFF_SHORT 200
#define LED_OFF_MED 1000
#define LED_OFF_LONG 2000

#define LAUNCH_DETECT_THRESHOLD 2 // Meters
#define LAUNCH_DETECT_TICKS 300     // This many ticks for confirmed launch

#define DESCENT_DETECT_THRESHOLD -2
#define DESCENT_DETECT_TICKS 400

#define LANDED_DETECT_THRESHOLD_LOW -0.4 // Meters
#define LANDED_DETECT_THRESHOLD_HIGH 0.4 // Meters
#define LANDED_DETECT_TICKS 5000         // Number of ticks with subthreshold altitude change for landing event

#define LOG_INTERVAL 1000 // ms between (minimal) data log

const unsigned long idleLedPattern[] = {
    LED_ON_SHORT,
    LED_OFF_SHORT,
    LED_ON_SHORT,
    LED_OFF_LONG,
};

const unsigned long calibrateLedPattern[] = {
    LED_ON_MED, LED_OFF_MED};

const unsigned long armedLedPattern[] = {
    LED_ON_SHORT, LED_OFF_SHORT};

const unsigned long groundLedPattern[] = {
    LED_ON_LONG,
    LED_OFF_MED,
    LED_ON_SHORT,
    LED_OFF_LONG,
};

MPU6050 accelgyro;   // I2C MPU6050 IMU
Adafruit_BMP280 bmp; // I2C bmp280 barometer for altitude

bool mpu_state = false;
bool bmp_state = false;
// IMU offsets
int useImuOffsets = false;
int axOffset = 0.0;
int ayOffset = 0.0;
int azOffset = 0.0;
int gxOffset = 0.0;
int gyOffset = 0.0;
int gzOffset = 0.0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float pressure, bmpTemperature, altitude, previousAltitude;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
float deltaAltitude = 0;

float peakAltitude = 0; // The largest altitude seen so far (locked at descent)

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

char *StateNames[] = {"IDLE", "CALIBRATE", "ARMED", "ASCENT", "DESCENT", "LANDED", "GROUNDSTATION", "ERROR"};

enum Command
{
    // Settings related
    TOGGLE_DUAL_DEPLOY, // 0
    SET_DD_MAIN_ALT, // 1
    TOGGLE_DROUGE_ENABLED, // 2
    TOGGLE_DEBUG,  // Toggle debug mode 3
    UNARM,         // Return to idle (from armed) 4
    ARM,           // Move to armed (from calibrate) 5
    ENTER_CALIBRATE,     // Move to calibrate (from idle) 6
    REPORT,        // Send systems report 7
    SYSTEMCHECK,   // Perform systems check 8
    TOGGLESOUND,   // Toggle buzzer manually 9
    LOCATE,        // Report location only (gps coords) 10 
    ENTER_GROUNDSTATION, // Enter GS mode (from any non flight mode) 11
    ABORT,         // CATO/ perform emergancy procedures (depends on state) 12
    NONE           // No command (default) 
};

State state = State::IDLE;
int stateChanged = 0; // Has the state changed since last tick(); (0 - no, 1 - yes, 2 - state changing this tick (ignore))

uint16_t launchDetectTicker = 0;
uint16_t descentTicker = 0;
uint16_t landedTicker = 0;

Command activeCommand = Command::NONE;
unsigned long commandData = 0; // Data passed with command

uint16_t mainDeploymentAltitude = 0; // Altitude at which main chute deployed (0 to disable)
bool drougeChuteEnabled = false;     // If true fire DrougeCH pyro at detect apogee
bool dualDeploymentEnabled = false;  // If true wait for mainDeploymentAltitude and then fire DDMainCH

bool drougeFired = false;    // set true after drouge pyro fired
bool mainChuteFired = false; // Set true after dual deploy main chute fired
bool stageChFired = false;   // seccond stage motor has been ignited)

bool masterArm = false; // Set true when entering arm state, false when unarmed, landed or ground station entered

unsigned long lastLedEvent = 0;
int activeLedPattern = 0;
uint16_t ledPatternStage = 0;

bool buzzerState = false;

float initalPresure = 0;                                                      // Presure at launch pad
unsigned long launchEventTimestamp, apoggeeEventTimestamp, landingEventTimestamp = 0; // Time at which the events were detected

unsigned long lastLogEvent = 0;
// Function prototypes
void initSensors();
void initErrorLoop();

// Tasks
void logData();
void pollImu();
void pollBmp();

void minimalLog();
void readCmd();
void updateOutputs();
void tick();
void systemCheck();
void stateChange(State newState);
bool firePyroCH(uint8_t channel);

void setup();
void loop();
#endif
