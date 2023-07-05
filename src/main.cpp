#include "I2Cdev.h"
#include "MPU6050.h"
#include "FS.h"
#include "LittleFS.h" // Used to store flight data in EEPROM/Flash
// Wifi and OTA
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoOTA.h>
#include <Adafruit_BMP280.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define DEBUG // Comment out to disable debug messages
#define VERSION "0.1.1"
#define FORMAT_LITTLEFS_IF_FAILED true

#define SDA 0
#define SCL 2

const int ramBufferLength = 100;
const int flushInterval = 90; // flush to FS every x entries
const char *metadataFilename = "/metadata.csv";
const char *filenameBase = "/flight_";
const char *filenameExt = ".csv";
const char *ssid = "24hr team blonde";              // Wifi SSID
const char *password = "117-Slightly-hot-chIllys*"; // Wifi password

// IMU offsets
const int useImuOffsets = false;
const int axOffset = 0.0;
const int ayOffset = 0.0;
const int azOffset = 0.0;
const int gxOffset = 0.0;
const int gyOffset = 0.0;
const int gzOffset = 0.0;

MPU6050 accelgyro;   // I2C MPU6050 IMU
Adafruit_BMP280 bmp; // I2C bmp280 barometer for altitude

int16_t ax, ay, az;
int16_t gx, gy, gz;
uint16_t pressure, altitude;

bool mpu_state = false;
bool wifi_state = false;
bool bmp_state = false;

int ramBuffer[ramBufferLength][6]; // 6 columns for ax, ay, az, gx, gy, gz
int ramBufferIndex = 0;
int entriesSinceLastFlush = 0;
uint32_t flightIdNumber = 0; // Loaded from metadata file on startup and incremented on each flight

uint16_t accelerations[ramBufferLength][3];
float velocities[ramBufferLength][3];
float altitudes[ramBufferLength];

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

void load_flight_id()
{
// Loads flight ID from metadata file
// Metadata file should be in format:
// entry_name=value; (e.g. flight_id=0;) followed by a newline
#ifdef DEBUG
    Serial.println("(D) Loading flight ID");
#endif
    // We need to read and write to the metadata file
    File metadataFile = LittleFS.open(metadataFilename, "r+");
    if (!metadataFile)
    {
// Metadata file doesn't exist, create it
#ifdef DEBUG
        Serial.println("(D) Metadata file doesn't exist, creating it");
#endif
        metadataFile = LittleFS.open(metadataFilename, "r+");
        metadataFile.println("flight_id=0;");
        metadataFile.close();
        flightIdNumber = 0;
        return;
    }
    // Find flight_id entry
    String entry;
    while (metadataFile.available())
    {
        entry = metadataFile.readStringUntil(';');
#ifdef DEBUG
        Serial.print("(D) Found entry: ");
        Serial.println(entry);
#endif
        if (entry.startsWith("flight_id="))
        {
            // Found flight_id entry
            String flightIdNumberStr = entry.substring(10);
            flightIdNumber = (uint32_t)flightIdNumberStr.toInt();
// Increment flight ID in metadata file
#ifdef DEBUG
            Serial.print("(D) Incrementing flight ID to ");
            Serial.println(flightIdNumber + 1);
#endif
            metadataFile.seek(0);
            metadataFile.print("flight_id=");
            metadataFile.print(flightIdNumber + 1);
            metadataFile.println(";");
            break;
        }
    }
    metadataFile.close();
#ifdef DEBUG
    Serial.print("(D) Flight ID is ");
    Serial.println(flightIdNumber);
#endif
}

void create_flight_file()
{
// Creates new flight file with name flight_<flight_id>.csv
// Flight ID is loaded from metadata file
#ifdef DEBUG
    Serial.println("(D) Creating flight file");
#endif
    String filename = filenameBase;
    filename += String(flightIdNumber);
    filename += filenameExt;
    File flightFile = LittleFS.open(filename, "w");
    flightFile.println("ax,ay,az,gx,gy,gz");
    flightFile.close();
}

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(SDA, SCL);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    Serial.begin(115200);
    Serial.println("CFlight v" + String(VERSION));
    Serial.println("Initializing WiFi...");
    Serial.print("- Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    uint32_t start = millis();
    wifi_state = true;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        if (millis() - start > 10000)
        {
            Serial.println("- Connection timed out (Continuing without WiFi)");
            wifi_state = false;
        }
    }
    if (wifi_state)
    {
        Serial.println("- Connected to WiFi");
        Serial.println("- IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println("Initializing OTA...");
        ArduinoOTA.setHostname("cflight");
        ArduinoOTA.begin();
        Serial.println("- OTA initialized, agent name: cflight");
    }
    else
    {
        Serial.println("- Failed to connect to WiFi");
    }
    // initialize devices
    Serial.println("Initializing I2C devices...");
    Serial.println("(IMU):");
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(3); // +-16g accel scale

    // verify connection
    mpu_state = accelgyro.testConnection();
    if (mpu_state)
    {
        Serial.println("- Connected to MPU6050 (AxGy)");
        if (useImuOffsets)
        {
            Serial.println("- Setting IMU offsets...");
            accelgyro.setXAccelOffset(axOffset);
            accelgyro.setYAccelOffset(ayOffset);
            accelgyro.setZAccelOffset(azOffset);
            accelgyro.setXGyroOffset(gxOffset);
            accelgyro.setYGyroOffset(gyOffset);
            accelgyro.setZGyroOffset(gzOffset);
        }
    }
    else
    {
        Serial.println("- Failed to connect to MPU6050!");
        while (1)
        {
            delay(1);
        }
    }
    Serial.println("(BMP):");
    bmp_state = bmp.begin();
    if (!bmp_state)
    {
        Serial.println("- Failed to connect to BMP280!");
        while (1)
        {
            delay(1);
        }
    }
    Serial.println("- Connected to BMP, configuring");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial.println("- Done! BMP online");

    Serial.println("Initializing FileSystem...");
    if (!LittleFS.begin())
    {
        Serial.println("- Failed to mount file system");
        while (1)
        {
            delay(1);
        }
    }
    Serial.println("- Mounted file system");
    // Check if metadata file exists
    if (!LittleFS.exists(metadataFilename))
    {
        Serial.println("- Metadata file does not exist, creating...");
        File metadataFile = LittleFS.open(metadataFilename, "w");
        metadataFile.print("flight_id=");
        metadataFile.println("0;");
        metadataFile.close();
    }
    else
    {
        Serial.println("- Metadata file exists");
    }
    // Load flight ID from metadata file
    load_flight_id();
    Serial.print("- Flight ID: ");
    Serial.println(flightIdNumber);
    // Create new flight file
    create_flight_file();
    Serial.println("- Created new flight file");
    previousTime = millis();
}

void flush_buffer_to_file()
{
// Flushes ramBuffer to flightFile and resets ramBufferIndex
#ifdef DEBUG
    Serial.println("(D) Opening flight file");
    uint32_t start = micros();
#endif
    String filename = filenameBase;
    filename += String(flightIdNumber);
    filename += filenameExt;
    File flightFile = LittleFS.open(filename, "a");
#ifdef DEBUG
    Serial.print("(D) Opened file in ");
    Serial.print(micros() - start);
    Serial.println(" us");
    start = micros();
#endif
    for (int i = 0; i < ramBufferIndex; i++)
    {
        flightFile.print(ramBuffer[i][0]);
        flightFile.print(",");
        flightFile.print(ramBuffer[i][1]);
        flightFile.print(",");
        flightFile.print(ramBuffer[i][2]);
        flightFile.print(",");
        flightFile.print(ramBuffer[i][3]);
        flightFile.print(",");
        flightFile.print(ramBuffer[i][4]);
        flightFile.print(",");
        flightFile.print(ramBuffer[i][5]);
        flightFile.print("\n");
    }
    ramBufferIndex = 0;
    entriesSinceLastFlush = 0;
    flightFile.close();
#ifdef DEBUG
    Serial.print("(D) Flushed buffer to file in ");
    Serial.print(micros() - start);
    Serial.println(" us");
#endif
}

void append_readings()
{
    // Adds latest values (in ax, ay, az etc varibles) to in RAM buffer of values
    ramBuffer[ramBufferIndex][0] = ax;
    ramBuffer[ramBufferIndex][1] = ay;
    ramBuffer[ramBufferIndex][2] = az;
    ramBuffer[ramBufferIndex][3] = gx;
    ramBuffer[ramBufferIndex][4] = gy;
    ramBuffer[ramBufferIndex][5] = gz;
    ramBufferIndex++;
    entriesSinceLastFlush++;
    if (ramBufferIndex >= ramBufferLength)
    {
        ramBufferIndex = 0;
    }
}

void processImu()
{
    accelerations[ramBufferIndex][0] = ax;
    accelerations[ramBufferIndex][1] = ay;
    accelerations[ramBufferIndex][2] = az;
    // Integrate for velocity
    velocities[ramBufferIndex][0] = ax * (deltaTime / 1000);
    velocities[ramBufferIndex][1] = ay * (deltaTime / 1000);
    velocities[ramBufferIndex][2] = az * (deltaTime / 1000);
    ramBufferIndex++;
}

void debugPrintReadings()
{
    Serial.print("(D) data:");
    Serial.print(deltaTime);
    Serial.print(",");
    // Accelerations
    Serial.print(accelerations[ramBufferIndex - 1][0]);
    Serial.print(",");
    Serial.print(accelerations[ramBufferIndex - 1][1]);
    Serial.print(",");
    Serial.print(accelerations[ramBufferIndex - 1][2]);
    Serial.print(",");
    // Velocities
    Serial.print(velocities[ramBufferIndex - 1][0]);
    Serial.print(",");
    Serial.print(velocities[ramBufferIndex - 1][1]);
    Serial.print(",");
    Serial.print(velocities[ramBufferIndex - 1][2]);
}

void loop()
{
    currentTime = millis();
    deltaTime = currentTime - previousTime;
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    processImu();
#ifdef DEBUG
    debugPrintReadings()
#endif
}
