// #define SPEED_TEST
#ifdef SPEED_TEST
// Simple speed test for filesystem objects
// Released to the public domain by Earle F. Philhower, III

#include <FS.h>
#include <LittleFS.h>

// Choose the filesystem to test
// WARNING:  The filesystem will be formatted at the start of the test!

#define TESTFS LittleFS
// #define TESTFS SPIFFS
// #define TESTFS SDFS

// How large of a file to test
#define TESTSIZEKB 50

// Format speed in bytes/second.  Static buffer so not re-entrant safe
const char *rate(unsigned long start, unsigned long stop, unsigned long bytes)
{
    static char buff[64];
    if (stop == start)
    {
        strcpy_P(buff, PSTR("Inf b/s"));
    }
    else
    {
        unsigned long delta = stop - start;
        float r = 1000.0 * (float)bytes / (float)delta;
        if (r >= 1000000.0)
        {
            sprintf_P(buff, PSTR("%0.2f MB/s"), r / 1000000.0);
        }
        else if (r >= 1000.0)
        {
            sprintf_P(buff, PSTR("%0.2f KB/s"), r / 1000.0);
        }
        else
        {
            sprintf_P(buff, PSTR("%d bytes/s"), (int)r);
        }
    }
    return buff;
}

void DoTest(FS *fs)
{
    if (!fs->format())
    {
        Serial.printf("Unable to format(), aborting\n");
        return;
    }
    if (!fs->begin())
    {
        Serial.printf("Unable to begin(), aborting\n");
        return;
    }

    uint8_t data[256];
    for (int i = 0; i < 256; i++)
    {
        data[i] = (uint8_t)i;
    }

    Serial.printf("Creating %dKB file, may take a while...\n", TESTSIZEKB);
    unsigned long start = millis();
    File f = fs->open("/testwrite.bin", "w");
    if (!f)
    {
        Serial.printf("Unable to open file for writing, aborting\n");
        return;
    }
    for (int i = 0; i < TESTSIZEKB; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            f.write(data, 256);
        }
    }
    f.close();
    unsigned long stop = millis();
    Serial.printf("==> Time to write %dKB in 256b chunks = %lu milliseconds\n", TESTSIZEKB, stop - start);

    f = fs->open("/testwrite.bin", "r");
    Serial.printf("==> Created file size = %zu\n", f.size());
    f.close();

    Serial.printf("Reading %dKB file sequentially in 256b chunks\n", TESTSIZEKB);
    start = millis();
    f = fs->open("/testwrite.bin", "r");
    for (int i = 0; i < TESTSIZEKB; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            f.read(data, 256);
        }
    }
    f.close();
    stop = millis();
    Serial.printf("==> Time to read %dKB sequentially in 256b chunks = %lu milliseconds = %s\n", TESTSIZEKB, stop - start, rate(start, stop, TESTSIZEKB * 1024));

    Serial.printf("Reading %dKB file MISALIGNED in flash and RAM sequentially in 256b chunks\n", TESTSIZEKB);
    start = millis();
    f = fs->open("/testwrite.bin", "r");
    f.read();
    for (int i = 0; i < TESTSIZEKB; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            f.read(data + 1, 256);
        }
    }
    f.close();
    stop = millis();
    Serial.printf("==> Time to read %dKB sequentially MISALIGNED in flash and RAM in 256b chunks = %lu milliseconds = %s\n", TESTSIZEKB, stop - start, rate(start, stop, TESTSIZEKB * 1024));

    Serial.printf("Reading %dKB file in reverse by 256b chunks\n", TESTSIZEKB);
    start = millis();
    f = fs->open("/testwrite.bin", "r");
    for (int i = 0; i < TESTSIZEKB; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (!f.seek(256 + 256 * j * i, SeekEnd))
            {
                Serial.printf("Unable to seek to %d, aborting\n", -256 - 256 * j * i);
                return;
            }
            if (256 != f.read(data, 256))
            {
                Serial.printf("Unable to read 256 bytes, aborting\n");
                return;
            }
        }
    }
    f.close();
    stop = millis();
    Serial.printf("==> Time to read %dKB in reverse in 256b chunks = %lu milliseconds = %s\n", TESTSIZEKB, stop - start, rate(start, stop, TESTSIZEKB * 1024));

    Serial.printf("Writing 64K file in 1-byte chunks\n");
    start = millis();
    f = fs->open("/test1b.bin", "a");
    for (int i = 0; i < 65536; i++)
    {
        f.write((uint8_t *)&i, 1);
    }
    f.close();
    stop = millis();
    Serial.printf("==> Time to write 64KB in 1b chunks = %lu milliseconds = %s\n", stop - start, rate(start, stop, 65536));

    Serial.printf("Reading 64K file in 1-byte chunks\n");
    start = millis();
    f = fs->open("/test1b.bin", "r");
    for (int i = 0; i < 65536; i++)
    {
        char c;
        f.read((uint8_t *)&c, 1);
    }
    f.close();
    stop = millis();
    Serial.printf("==> Time to read 64KB in 1b chunks = %lu milliseconds = %s\n", stop - start, rate(start, stop, 65536));

    start = millis();
    auto dest = fs->open("/test1bw.bin", "w");
    f = fs->open("/test1b.bin", "r");
    auto copysize = f.sendAll(dest);
    dest.close();
    stop = millis();
    Serial.printf("==> Time to copy %d = %zd bytes = %lu milliseconds = %s\n", f.size(), copysize, stop - start, rate(start, stop, f.size()));
    f.close();
}

void setup()
{
    Serial.begin(115200);
    Serial.printf("Beginning test\n");
    Serial.flush();
    DoTest(&TESTFS);
    Serial.println("done");
}

void loop()
{
    delay(10000);
}
#else
#include "I2Cdev.h"
#include "MPU6050.h"
#include "FS.h"
#include "LittleFS.h" // Used to store flight data in EEPROM/Flash
#include <Adafruit_BMP280.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define DEBUG // Comment out to disable debug messages
#define VERSION "0.1.1"

#define SDA 0
#define SCL 2

const int flushInterval = 50; // flush to FS every x entries
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

bool mpu_state = false;
bool wifi_state = false;
bool bmp_state = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float pressure, bmpTemperature;

uint32_t flightIdNumber = 1; // Loaded from metadata file on startup and incremented on each flight
File flightFile;

int entriesSinceLastFlush = 0;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

bool recordState = false; // Set to true to start recording data

void log_filesystem_info()
{
    Serial.println("- Filesystem info:");
    FSInfo fs_info;
    LittleFS.info(fs_info);
    Serial.print(" Total bytes: ");
    Serial.println(fs_info.totalBytes);
    Serial.print(" Used bytes: ");
    Serial.println(fs_info.usedBytes);
    Serial.print(" Block size: ");
    Serial.println(fs_info.blockSize);
    Serial.print(" Page size: ");
    Serial.println(fs_info.pageSize);
    Serial.print(" Max open files: ");
    Serial.println(fs_info.maxOpenFiles);
    Serial.print(" Max path length: ");
    Serial.println(fs_info.maxPathLength);
    // Print metadata file contents
    Serial.println("- Metadata file contents:");
    File metadataFile = LittleFS.open(metadataFilename, "r");
    metadataFile.seek(0);
    while (metadataFile.available())
    {
        Serial.write(metadataFile.read());
    }
    metadataFile.close();
    // Print file list
    Serial.println("- File list:");
    Dir dir = LittleFS.openDir("/");
    while (dir.next())
    {
        Serial.print(dir.fileName());
        Serial.print(" - ");
        Serial.println(dir.fileSize());
    }
}

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
        metadataFile.println("flight_id=2;");
        metadataFile.close();
        flightIdNumber = 1;
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
    flightFile = LittleFS.open(filename, "w");
    flightFile.println("ax,ay,az,gx,gy,gz");
    flightFile.close();
    flightFile = LittleFS.open(filename, "a");
}

void setup()
{
    // initialize serial communication
    Serial.begin(115200);
    Serial.println("CFlight v" + String(VERSION));
    // Serial.println("Initializing WiFi...");
    // Serial.print("- Connecting to ");
    // Serial.println(ssid);
    // WiFi.begin(ssid, password);
    // uint32_t start = millis();
    // wifi_state = true;
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     Serial.print(".");
    //     if (millis() - start > 10000)
    //     {
    //         Serial.println("- Connection timed out (Continuing without WiFi)");
    //         wifi_state = false;
    //     }
    // }
    // if (wifi_state)
    // {
    //     Serial.println("- Connected to WiFi");
    //     Serial.println("- IP address: ");
    //     Serial.println(WiFi.localIP());
    //     Serial.println("Initializing OTA...");
    //     ArduinoOTA.setHostname("cflight");
    //     ArduinoOTA.begin();
    //     Serial.println("- OTA initialized, agent name: cflight");
    // }
    // else
    // {
    //     Serial.println("- Failed to connect to WiFi");
    // }
    // initialize devices
    Serial.println("Initializing I2C bus & devices...");
    Wire.begin(SDA, SCL);
    Serial.println("- Joined I2C bus");
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
    bmp_state = bmp.begin(BMP280_ADDRESS, BMP280_CHIPID);
    if (!bmp_state)
    {
        Serial.print("- Failed to connect to BMP280! SensorID: 0x");
        Serial.println(bmp.sensorID(), 16);
        Serial.print("- Trying alternate I2C address 0x");
        Serial.println(BMP280_ADDRESS_ALT, 16);
        bmp_state = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
        if (!bmp_state)
        {
            Serial.println("- Failed to connect to BMP280!");
            while (1)
            {
                delay(1);
            }
        }
        else
        {
            Serial.print("- Connected to BMP280! SensorID: 0x");
            Serial.print(bmp.sensorID(), 16);
            Serial.println(" (Alternate address)");
        }
    }
    else
    {
        Serial.print("- Connected to BMP280! SensorID: 0x");
        Serial.print(bmp.sensorID(), 16);
        Serial.println(" (Default address)");
    }
    Serial.println("- Setting BMP280 parameters...");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

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
        metadataFile.println("2;");
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
    log_filesystem_info();
    Serial.println("- Creating flight file...");
    // Create new flight file
    create_flight_file();
    Serial.println("- Created new flight file");
    previousTime = millis();
}

void flush_buffer_to_file()
{
// Flushes ramBuffer to flightFile and resets ramBufferIndex
#ifdef DEBUG
    uint32_t start = micros();
#endif
    flightFile.flush();
    entriesSinceLastFlush = 0;
#ifdef DEBUG
    Serial.print("(D) Flushed buffer to file in ");
    Serial.print(micros() - start);
    Serial.println(" us");
#endif
}

void appendToFile()
{
    flightFile.print(currentTime);
    flightFile.printf(",%i,%i,%i,%i,%i,%i,%f,%f\n",
                      ax, ay, az,
                      gx, gy, gz,
                      pressure, bmpTemperature);
    entriesSinceLastFlush++;
}

void handleIMU()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void handleBMP()
{
    pressure = bmp.readPressure();
    bmpTemperature = bmp.readTemperature();
}

void loop()
{
    currentTime = millis();
    deltaTime = currentTime - previousTime;
    // read raw accel/gyro measurements from device
    handleIMU();
    handleBMP();
    if (recordState)
    {
        appendToFile();
        if (entriesSinceLastFlush >= flushInterval)
        {
            flush_buffer_to_file();
        }
    }

    if (Serial.available() > 0)
    {
        char command = Serial.read();
        if (command == 'r')
        {
            recordState = !recordState;
            if (recordState)
            {
                Serial.println("Recording started");
            }
            else
            {
                Serial.println("Recording stopped");
            }
        }
        else if (command == 'b' && recordState)
        {
            Serial.println("Reading flight file to serial...");
            recordState = false;
            flightFile.flush();
            flightFile.close();
            String filename = filenameBase;
            filename += String(flightIdNumber);
            filename += filenameExt;
            File flightFileRB = LittleFS.open(filename, "r");
            flightFileRB.seek(0);
            Serial.println("<BOF>");
            while (flightFileRB.available())
            {
                Serial.write(flightFileRB.read());
            }
            flightFileRB.close();
            flightFile = LittleFS.open(filename, "a");
            Serial.println("<EOF>");
        }
        else if (command == 'e' && !recordState)
        {
            recordState = false;
            Serial.println("Erasing filesystem...");
            LittleFS.format();
            Serial.println("Done! Rebooting...");
            ESP.restart();
        }
        else if (command == 'i')
        {
            // Print filesystem info (eg. used space, free space, etc.)
            log_filesystem_info();
        } else if (command == 'd' && !recordState) {
            // Download/ print to serial a flight file
            Serial.println("Enter flight ID to download (0 for latest):");
            Serial.print("List of flights: ");
            // List all files in the root
            Dir dir = LittleFS.openDir("/");
            while (dir.next())
            {
                Serial.print(dir.fileName());
                Serial.print(" ");
            }
            Serial.println();
            Serial.print("> ");
            while (Serial.available() == 0)
            {
                delay(1);
            }
            String flightIdString = Serial.readStringUntil('\n');
            int flightId = flightIdString.toInt();
            if (flightId == 0) {
                flightId = flightIdNumber;
            }
            Serial.print("Downloading flight ");
        }
    }
}

#endif