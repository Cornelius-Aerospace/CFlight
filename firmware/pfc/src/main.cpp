#include "main.h"
#ifdef SD_CARD
void listDir(const char *dirname, uint8_t levels)
{
    printlogf("Listing directory: %s\n", dirname);

    File root = SD.open(dirname);
    if (!root)
    {
        printlnlog("- Failed to open directory!");
        return;
    }
    if (!root.isDirectory())
    {
        printlnlog("- Not a directory!");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            printlog("  DIR : ");
            printlnlog(file.name());
            if (levels)
            {
                listDir(file.path(), levels - 1);
            }
        }
        else
        {
            printlog("  FILE: ");
            printlog(file.name());
            printlog("  SIZE: ");
            printlnlog(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(const char *path)
{
    printlogf("Creating Dir: %s\n", path);
    if (SD.mkdir(path))
    {
        printlnlog("- Dir created");
    }
    else
    {
        printlnlog("- mkdir failed!");
    }
}

void readFile(const char *path)
{
    printlogf("Reading file: %s\n", path);

    File file = SD.open(path);
    if (!file)
    {
        printlnlog("- Failed to open file for reading!");
        return;
    }

    printlog("- Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(const char *path, const char *message)
{
    printlogf("Writing file: %s\n", path);

    File file = SD.open(path, FILE_WRITE);
    if (!file)
    {
        printlnlog("- Failed to open file for writing!");
        return;
    }
    if (file.print(message))
    {
        printlnlog("- File written");
    }
    else
    {
        printlnlog("- Write failed!");
    }
    file.close();
}

void appendFile(const char *path, const char *message)
{
    printlogf("Appending to file: %s\n", path);

    File file = SD.open(path, FILE_APPEND);
    if (!file)
    {
        printlnlog("- Failed to open file for appending!");
        return;
    }
    if (file.print(message))
    {
        printlnlog("- Message appended");
    }
    else
    {
        printlnlog("- Append failed!");
    }
    file.close();
}

uint16_t createFlightFiles(uint16_t flight_id)
{
    String pathName = "/" + String(flight_id);

    if (SD.exists(pathName + "/flight.log"))
    {
        printlnlog("Error, flight ID already used, trying next");
        return createFlightFiles(flight_id + 1);
    }
    createDir(pathName.c_str());
    pathName += "/";
    logFile = SD.open(pathName + "flight.log", FILE_APPEND);
    logFile.printf("Flight %i log started\n", flight_id);
    dataFile = SD.open(pathName + "flight.csv", FILE_APPEND);
    dataFile.println(CSV_HEADER);
    eventsFile = SD.open(pathName + "flight.events", FILE_APPEND);
    eventsFile.println("Flight files initalised");
    flightFilesReady = true;
    logFile.flush();
    dataFile.flush();
    eventsFile.flush();
    return flight_id;
}

void closeFlightFiles()
{
    printlnlog("Closing flight files");
    logFile.close();
    eventsFile.close();
    dataFile.close();
    flightFilesReady = false;
}
void initSD()
{
    printlnlog("Init SD card");
    if (!SD.begin(5, SPI, 4000000U, "/sd", 5, true))
    {
        printlnlog("- Card Mount Failed!");
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        printlnlog("- No SD card attached!");
        return;
    }

    printlog("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        printlnlog("MMC");
    }
    else if (cardType == CARD_SD)
    {
        printlnlog("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        printlnlog("SDHC");
    }
    else
    {
        printlnlog("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    printlogf("SD Card Size: %lluMB\n", cardSize);
    printlogf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    printlogf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    sdReady = true;
}

#endif
#ifdef COMM_TEST
void setup()
{
    Serial.begin(115200);
    initComms();
    testComms();
    while (1)
    {
    }
}

void loop()
{
}
#else
void allocateHistoryMemory()
{
    printlnlog("Allocating history buffers:");
    printlog("- Allocating ");
    printlog(HISTORY_SIZE * LOG_SENSOR_COUNT * sizeof(float));
    printlog(" Bytes for history buffers with: ");
    printlog(LOG_SENSOR_COUNT);
    printlnlog(" sensors");
    printlogf("- available heap before allocating %i\n", ESP.getFreeHeap());
    printlogf("- biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    altitudeHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    speedHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    printlnlog("Allocated history buffers");
    printlogf("- available heap after allocating %i\n", ESP.getFreeHeap());
    printlogf("- biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

void updateAccelOffset(SystemSettings settings)
{
    if (settings.useImuOffsets)
    {
        printlnlog("(SystemSettings Change): Updating IMU offsets");
        accelgyro.setXAccelOffset(settings.axOffset);
        accelgyro.setYAccelOffset(settings.ayOffset);
        accelgyro.setZAccelOffset(settings.azOffset);
        accelgyro.setXGyroOffset(settings.gxOffset);
        accelgyro.setYGyroOffset(settings.gyOffset);
        accelgyro.setZGyroOffset(settings.gzOffset);
    }
}

void updateSystemSettings(SystemSettings newSystemSettings)
{
    printlnlog("System Settings Updated!");
    SysSettings = newSystemSettings;
    updateAccelOffset(SysSettings);
}

void initSensors()
{
#ifndef SIM_MODE
    printlnlog("Initializing I2C bus & devices...");
    Wire.begin(SDA, SCL);
    printlnlog("- Joined I2C bus");
    printlnlog("(IMU):");
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(3); // +-16g accel scale

    // verify connection
    mpu_state = accelgyro.testConnection();
    if (mpu_state)
    {
        printlnlog("- Connected to MPU6050 (AxGy)");
        if (SysSettings.useImuOffsets)
        {
            printlnlog("- Setting IMU offsets...");
            accelgyro.setXAccelOffset(SysSettings.axOffset);
            accelgyro.setYAccelOffset(SysSettings.ayOffset);
            accelgyro.setZAccelOffset(SysSettings.azOffset);
            accelgyro.setXGyroOffset(SysSettings.gxOffset);
            accelgyro.setYGyroOffset(SysSettings.gyOffset);
            accelgyro.setZGyroOffset(SysSettings.gzOffset);
        }
    }
    else
    {
        printlnlog("- Failed to connect to MPU6050!");
        initErrorLoop(SysSettings);
    }
    printlnlog("(BMP):");
    bmp_state = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    if (!bmp_state)
    {
        printlnlog("- Failed to connect to BMP280!");
        initErrorLoop(SysSettings);
    }
    else
    {
        printlog("- Connected to BMP280! SensorID: 0x");
        printlogf("%h", bmp.sensorID());
    }
    printlnlog("- Setting BMP280 parameters...");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    printlnlog("- Done! BMP online");
#ifdef GPS_CONNECTED
    printlnlog("Connecting to GPS...");
    Serial2.begin(9600, SERIAL_8N1, SysSettings.gpsRxPin, SysSettings.gpsTxPin);
    printlnlog("- Done! GPS online");
#endif
#endif
}

void pollGps()
{
#ifndef SIM_MODE
#ifdef GPS_CONNECTED
    while (Serial2.available())
        gps.encode(Serial2.read());

    if (gps.satellites.isValid() && gps.satellites.isUpdated())
    {
        gpsSatilliteCount = gps.satellites.value();
    }

    if (gps.location.isValid() && gps.location.isUpdated())
    {
        gpsLatitude = gps.location.lat();
        gpsLongitude = gps.location.lng();
        gpsFix = true;
    }

    if (gps.altitude.isValid() && gps.altitude.isUpdated())
    {
        gpsAltitude = gps.altitude.meters();
    }

    if (gps.speed.isValid() && gps.speed.isUpdated())
    {
        gpsSpeed = gps.speed.kmph();
    }

    if (gps.hdop.isValid() && gps.hdop.isUpdated())
    {
        gpsHdop = gps.hdop.hdop();
    }
    if (currentTime > 5000 && gps.charsProcessed() < 10)
    {
        // printlnlog("No GPS detected: check wiring.");
    }
#endif
#endif
}

uint16_t readFlightId()
{
#ifndef SD_CARD
    return 0;
#else
    if (SD.exists("/cflight.yaml"))
    {
        fs::File config = SD.open("/cflight.yaml", FILE_READ);
        String line = "";
        while (!line.startsWith("flightId:") && line != "EOF")
        {
            line = config.readStringUntil('\n');
        }
        if (line == "EOF")
        {
            return 0;
        }
        else
        {
            line.replace("flightId:", "");
            return line.toInt();
        }
    }
    else
    {
        saveMetaData();
        return 0;
    }
#endif
}

bool saveMetaData()
{
#ifdef SD_CARD
    fs::File file = SD.open("/cflight.yaml", FILE_WRITE);
    if (!file)
    {
        printlnlog("Can't open file for writing");
        return false;
    }
    file.printf("flightId:%i\n", flight_id);
    file.close();
#endif
    return true;
}

void setup()
{
    // initialize serial communication
    Serial.begin(115200);
    SysSettings = SystemSettings();
    printlnlog("CFlight v" + String(VERSION));
    pinMode(SysSettings.statusLedPin, OUTPUT);
    pinMode(SysSettings.errorLedPin, OUTPUT);
    pinMode(SysSettings.buzzerPin, OUTPUT);
// initialize devices
#ifdef SD_CARD
    initSD();
#endif
    initSensors();
    initComms();
    registerCmdPacketCallback(commandPacketCallback);
    registerTasks();
#ifdef RAM_LOG_ENABLED
    allocateHistoryMemory();
#endif
    flight_id = readFlightId() + 1;
    printlnlog("Boot complete!");
    previousTime = millis();
    currentTime = millis();
    lastSpeedMeasurement = millis();
#ifdef SIM_MODE
    commandCollector = "";
    printlnlog("Sim mode: awaiting packets");
    while (commandCollector != "START\n")
    {
        commandCollector += Serial.readString();
    }
    commandCollector = "";
    printlnlog("ACK");
    state = State::ARMED;
    saveMetaData();
    createFlightFiles(flight_id);
#endif
}

void registerTasks()
{
    xTaskCreate(detectEvents, "detectEvents", 2048, NULL, 2, &detectEventsTaskHandle);
    xTaskCreate(pollSensors, "pollSensors", 2048, NULL, 2, &pollSensorsTaskHandle);
    xTaskCreate(logData, "logData", 2048, NULL, 2, &logDataTaskHandle);
    xTaskCreate(updateOutputs, "updateOutputs", 2048, NULL, 1, &updateOutputsTaskHandle);
    xTaskCreate(systemReportTask, "systemReport", 2048, NULL, 1, &systemReportTaskHandle);
}

void pollSensors(void *param)
{
    for (;;)
    {
        pollImu();
        pollBmp();
#ifdef GPS_CONNECTED
        pollGps();
#endif
        vTaskDelay(SENSOR_POLL_INTERVAL);
        xTaskNotifyGiveIndexed(detectEventsTaskHandle, 0);
    }
}

void detectEvents(void *param)
{
    for (;;)
    {
        uint32_t xRes = ulTaskNotifyTakeIndexed(0, pdFALSE, portMAX_DELAY);
        if (xRes == 0)
        {
            // Timeout
            continue;
        }
        // Fresh sensor data is available
        if (stateChanged != 0)
            stateChanged -= 1;
        if (state == State::IDLE)
        {
            if (initalPresure == 0.0)
            {
                initalPresure = pressure; // First tick in cal mode, set initalPressure
            }
            else
            {
                calibrate_done = true; // ?
                // Add current to inital presure & divide by two ("running" avgerage) [TODO: check this]
                initalPresure += pressure;
                initalPresure /= 2;
            }
        }
        else if (state == State::ARMED)
        {

            // We are looking for a takeoff event
            if (altitude >= LAUNCH_DETECT_THRESHOLD) // TODO: also look at x axis of accelerometer crossing from negative to positive (ie gravity being counter-acted by acceleration by motor)
            {
                launchDetectTicker++;
                if (launchDetectTicker == 1)
                {
                    launchEventTimestamp = currentTime;
                    loggingData = true;
                }
                if (launchDetectTicker == LAUNCH_DETECT_TICKS)
                {
                    // Launch Event detected
                    stateChange(State::ASCENT);
                    loggingData = true;
                }
            }
            else
            {
                launchDetectTicker = 0;
                loggingData = false;
                logIndex = 0;
            }
        }
        else if (state == State::ASCENT)
        {
            // Are we still in-ascent
            if (altitude > peakAltitude)
            {
                peakAltitude = altitude;
                descentTicker = 0;
            }
            else if (peakAltitude - altitude >= LAUNCH_DETECT_THRESHOLD)
            {
                descentTicker++;
                if (descentTicker == 1)
                {
                    apoggeeEventTimestamp = currentTime;
                }
                if (descentTicker == LAUNCH_DETECT_TICKS)
                {
                    stateChange(State::DESCENT);
                }
            }
        }
        else if (state == State::DESCENT)
        {
            if (verticalVelocity >= LANDED_DETECT_THRESHOLD_LOW && verticalVelocity <= LANDED_DETECT_THRESHOLD_HIGH)
            {
                landedTicker++;
                if (landedTicker == 1)
                {
                    landingEventTimestamp = currentTime;
                }
                if (landedTicker == LANDED_DETECT_TICKS)
                {
                    stateChange(State::LANDED);
                }
            }
            else
            {
                landedTicker == 0;
            }
        }
        else if (state == State::LANDED)
        {
            printlnlog("Landing detected; Ending event detection & system report tasks");
            report(); // Print flight report
            vTaskSuspend(systemReportTaskHandle);
            vTaskSuspend(logDataTaskHandle);
            vTaskSuspend(detectEventsTaskHandle);
        }
    }
}

void logData(void *param)
{
    for (;;)
    {
        if (loggingData)
        {
#ifdef RAM_LOG_ENABLED
            altitudeHistory[logIndex] = altitude;
            speedHistory[logIndex] = verticalVelocity;
            logIndex++;
            if (logIndex >= HISTORY_SIZE)
            {
                printlnlog("Log full! Stopping flight log");
                loggingData = false;
            }
#endif
#ifdef SD_CARD
            if (flightFilesReady)
            {
                dataFile.printf("%i,%f,%f,%f,%f\n", currentTime, altitude, verticalVelocity, bmpTemperature, pressure);
            }
#endif
#ifdef FLUSH_EVERY
            flushCounterData++;
            if (flushCounterData >= FLUSH_EVERY)
            {
                dataFile.flush();
            }
#endif
        }
    }
    vTaskDelay(LOG_INTERVAL);
}

void pollImu()
{
#ifndef SIM_MODE
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#endif
}

void pollBmp()
{
#ifndef SIM_MODE
    pressure = bmp.readPressure();          // SI units (pascal)
    pressure /= 100;                        // Now in MPa
    bmpTemperature = bmp.readTemperature(); // Deg c
#endif
    deltaTime = lastSpeedMeasurement - millis();
    speedTicker += deltaTime;
    altitude = 44330 * (1.0 - pow(pressure / initalPresure, 0.1903)); // meters

    // Calculate vertical speed
    if (altitude > peakAltitude && isfinite(altitude))
    {
        peakAltitude = altitude;
        apoggeeEventTimestamp = currentTime;
    }
    if (speedTicker >= SPEED_INTERVAL)
    {
        // SPEED_INTERVAL has passed, calculate vertical speed
        lastSpeedMeasurement = currentTime;
        verticalVelocity = (altitude - previousAltitude); // Delta meters
        verticalVelocity /= speedTicker / 1000;

        previousAltitude = altitude;
        if (verticalVelocity > peakVerticalVelocity && isfinite(altitude))
        {
            peakVerticalVelocity = verticalVelocity;
            peakVerticalVelocityTime = currentTime;
        }
        speedTicker = 0;
    }
    speedTicker += deltaTime;
}

bool firePyroCH(uint8_t channel)
{
    if (!masterArm)
    {
        printlog("Refusing to fire pyro CH ");
        printlog(channel);
        printlnlog(", not armed!");
        return false;
    }
    // Master arm set, fire away!
    // [TODO: fire pyro logic]
    if (channel == 0)
    {
        // Drouge chute channel
        drougeFired = true;
    }
    else if (channel == 1)
    {
        // Main chute (dual deployment) channel
        mainChuteFired = true;
    }
    else if (channel == 2)
    {
        // Seccond stage motor channel
        stageChFired = true;
    }
    printlog("Pyro CH ");
    printlog(channel);
    printlnlog(" fired");
    return true;
}

void eventEntry(char *s)
{
#ifdef SD_CARD
    eventsFile.printf("[%s] %s", formatTimestamp(currentTime), s);
    eventsFile.println();

#ifdef FLUSH_EVERY
    flushCounterEvents++;
    if (flushCounterEvents >= FLUSH_EVERY)
    {
        eventsFile.flush();
    }
#endif
#endif
}

void stateChange(State newState)
{
    sprintf(fmtBuffer, "State change %s -> %s", StateNames[(uint8_t)state], StateNames[(uint8_t)newState]);
    printlnlog(fmtBuffer);
    eventEntry(fmtBuffer);
    fmtBuffer[0] = '\0';
    state = newState;
    stateChanged = 2; // State JUST changed
    // Update master arm
    if (newState == State::ARMED)
    {
        Serial.println("MFC: ARMED");
#ifdef SD_CARD
        saveMetaData();
        createFlightFiles(flight_id);
#endif
    }
    if (newState == State::LANDED || newState == State::CALIBRATE || newState == State::IDLE || newState == State::GROUNDSTATION)
    {
        masterArm = false;
    }
    else
    {
        masterArm = true;
    }
}

void systemCheck()
{
    printlnlog("Begining systems check:");
    // TODO systems check
}

void humanLogTimestamp(unsigned long timestamp)
{
    printlog(formatTimestamp(timestamp));
}

String formatTimestamp(unsigned long timestamp)
{
    unsigned long seconds = timestamp / 1000;
    unsigned long minutes = seconds / 60;
    timestamp %= 1000;
    seconds %= 60;
    minutes %= 60;
    char formattedTime[16]; // Buffer for the formatted time string

    // Format the time components into the desired string format with leading zeros
    snprintf(formattedTime, sizeof(formattedTime), "%02lu:%02lu.%03lu", minutes, seconds, timestamp);

    return String(formattedTime);
}

void report()
{
    printlnlog("Flight Report: ");
    printlog("Lift-off at: ");
    humanLogTimestamp(launchEventTimestamp);
    printlnlog();
    printlog("Apogee: ");
    printlog(peakAltitude);
    printlog("m, At time: ");
    humanLogTimestamp(apoggeeEventTimestamp);
    printlnlog();
    printlog("Max speed: ");
    printlog(peakVerticalVelocity);
    printlog("m/s, At time: ");
    humanLogTimestamp(currentTime);
    printlnlog();
    printlog("Touchdown at: ");
    humanLogTimestamp(landingEventTimestamp);
    printlnlog();
    printlog("Flight time: ");
    humanLogTimestamp(landingEventTimestamp - launchEventTimestamp);
    printlnlog();
#ifdef RAM_LOG_ENABLED
    printlog("Used: ");
    printlog((float)logIndex / (float)HISTORY_SIZE);
    printlnlog("% of RAM history buffer");
    printlnlog("-- (RAM) Flight History Data --");
    printlnlog("time,altitude,speed");
    for (uint8_t i = 0; i < finalLogIndex; i++)
    {
        printlog(i * LOG_INTERVAL);
        printlog(",");
        printlog(altitudeHistory[i]);
        printlog(",");
        printlog(speedHistory[i]);
        printlnlog();
    }
    printlnlog("-- End RAM Flight History Data --");
#endif
    printlnlog("End of report");
}

float cpuTemp()
{
    return (temprature_sens_read() - 32) / 1.8;
}

void systemReport()
{
    printlnlog("- System Report -");
    // printlogf("Operation Status: %s \n", operationStatus ? "OK" : "ERROR");
    printlogf("Uptime: %s, ", formatTimestamp(currentTime));
    printlogf("Errors: %i\n", errorCount);
#ifdef GPS_CONNECTED
    printlogf("GPS. Sat Fix: %s, Sat count: %i,", gpsFix ? "YES" : "NO", gpsSatilliteCount);
    printlogf(" HDOP: %i, ", gpsHdop);
    if (gpsFix)
    {
        printlogf("Lat: %f, Long: %f, Speed: %f", gpsLatitude, gpsLongitude, gpsSpeed);
    }
    printlnlog();
#endif

    printlogf("BMP. State: %s\n", bmp_state ? "OK" : "ERROR");
    printlogf(" Pressure: %fMPa, Temperature: %f°C\n", pressure, bmpTemperature);
    printlogf(" Altitude: %fm, Base Pressure: %fMPa\n", altitude, initalPresure);

    printlogf("IMU. State: %s\n ", mpu_state ? "OK" : "ERROR");
    printlogf("Ax: %i Ay: %i Az: %i\n", ax, ay, az);
    printlogf("Gx: %i Gy: %i Gz: %i\n", gx, gy, gz);

    printlogf("DEVICE. Cpu Temperature %f, State: %s\n", cpuTemp(), StateNames[state]);
#ifdef SD_CARD
    printlogf("SD card. State: %s \n", sdReady ? "READY" : "ERROR");
#endif
    // TODO: reset of stats
}

void systemReportTask(void *params)
{
    for (;;)
    {
        systemReport();
        vTaskDelay(1000);
    }
}

void updateOutputs(void *params)
{
    for (;;)
    {
        if (buzzerState)
        {
            tone(SysSettings.buzzerPin, 2000, 500);
        }
        else
        {
            digitalWrite(SysSettings.buzzerPin, LOW);
        }

        // Status LED
        if (stateChanged == 1)
        {
            ledPatternStage = 0;
            lastLedEvent = currentTime;
        }
        if (lastLedEvent == 0)
        {
            lastLedEvent = currentTime;
            ledPatternStage = 0;
            digitalWrite(SysSettings.statusLedPin, HIGH);
        }
        if (state == State::IDLE)
        {
            if (currentTime - lastLedEvent >= idleLedPattern[ledPatternStage])
            {
                lastLedEvent = currentTime;
                if (ledPatternStage % 2 == 0)
                {
                    digitalWrite(SysSettings.statusLedPin, LOW);
                }
                else
                {
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
                ledPatternStage++;
                if (ledPatternStage >= 4)
                {
                    ledPatternStage = 0;
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
            }
        }
        else if (state == State::CALIBRATE)
        {
            if (lastLedEvent == 0)
            {
                lastLedEvent = currentTime;
                digitalWrite(SysSettings.statusLedPin, HIGH);
            }
            else if (currentTime - lastLedEvent >= calibrateLedPattern[ledPatternStage])
            {
                lastLedEvent = currentTime;
                if (ledPatternStage == 0)
                {
                    ledPatternStage = 1;
                    digitalWrite(SysSettings.statusLedPin, LOW);
                }
                else
                {

                    ledPatternStage = 0;
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
            }
        }
        else if (state == State::ARMED)
        {
            if (lastLedEvent == 0)
            {
                lastLedEvent = currentTime;
                digitalWrite(SysSettings.statusLedPin, HIGH);
            }
            else if (currentTime - lastLedEvent >= armedLedPattern[ledPatternStage])
            {
                lastLedEvent = currentTime;
                if (ledPatternStage == 0)
                {
                    ledPatternStage = 1;
                    digitalWrite(SysSettings.statusLedPin, LOW);
                }
                else
                {

                    ledPatternStage = 0;
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
            }
        }
        else if (state == State::GROUNDSTATION)
        {
            if (lastLedEvent == 0)
            {
                lastLedEvent = currentTime;
                digitalWrite(SysSettings.statusLedPin, HIGH);
            }
            else if (currentTime - lastLedEvent >= groundLedPattern[ledPatternStage])
            {
                lastLedEvent = currentTime;
                if (ledPatternStage % 2 == 0)
                {
                    digitalWrite(SysSettings.statusLedPin, LOW);
                }
                else
                {
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
                ledPatternStage++;
                if (ledPatternStage >= 4)
                {
                    ledPatternStage = 0;
                    digitalWrite(SysSettings.statusLedPin, HIGH);
                }
            }
        }
        vTaskDelay(50);
    }
}

void saveFlight()
{
    // TODO: Implement!
    printlnlog("Unimplemented");
}

void minimalLog()
{
    printlogf("[%s] ",
              formatTimestamp(currentTime));
    printlogf("Alt: %fm, ", altitude);
    printlogf("dAlt: %fm/s, ", verticalVelocity);
    printlogf("State: %s\n", StateNames[state]);
}

bool commandPacketCallback(uint8_t *responsePacketBuffer, uint8_t *responsePacketLength, uint8_t *args, uint8_t argsCount, uint8_t argsArrayLength, Command cmd, unsigned long time, unsigned long salt)
{
    bool ackNack = false;
    uint8_t reason = 0; // 1: bad arg count/length, 2: bad arg(s) value, 3: Wrong context, 4: unknown/unsupported command type, 5+: reserved (0: ok)
    switch (cmd)
    {
    case Command::ARM:
        if (argsCount == 1 && argsArrayLength == 4)
        {
            unsigned long armCode = args[0] >> 24;
            armCode += args[1] >> 16;
            armCode += args[2] >> 8;
            armCode += args[3];
            // TODO: check arm code against current issued code - for now hardcoded to 12345 (ik, secure)
            if (armCode == TEMP_ARM_CODE)
            {
                stateChange(State::ARMED);
                ackNack = true;
            }
            else
                reason = 2;
        }
        else
        {
            reason = 1;
        }

        break;
    case Command::UNARM:
        if (state == State::ARMED)
        {
            stateChange(State::IDLE);
            ackNack = true;
        }
        else
        {
            reason = 3;
        }
        break;
    case Command::SET_BUZZER:
        if (argsCount == 1 && argsArrayLength == 1)
        {
            buzzerState = args[0] == 1;
            ackNack = true;
        }
        else
            reason = 1;
        break;
    case Command::SLEEP:
        if (argsCount == 1 && argsArrayLength == 4)
        {
            unsigned long masterCode = args[0] >> 24;
            masterCode += args[1] >> 16;
            masterCode += args[2] >> 8;
            masterCode += args[3];
            if (masterCode == MASTER_CODE)
            {
                stateChange(State::IDLE);
                // TODO: tell system to begin sleep procedure
                ackNack = true;
            }
            else
                reason = 2;
        }
        else
            reason = 1;
        break;
    case Command::POWER_DOWN:
        if (argsCount == 1 && argsArrayLength == 4)
        {
            unsigned long masterCode = args[0] >> 24;
            masterCode += args[1] >> 16;
            masterCode += args[2] >> 8;
            masterCode += args[3];
            if (masterCode == MASTER_CODE)
            {
                stateChange(State::IDLE);
                // TODO: tell system to shut down
                ackNack = true;
            }
            else
                reason = 2;
        }
        else
            reason = 1;
        break;

    case Command::SET_FLIGHT:
        if (argsCount == 4 && argsArrayLength == 4)
        {
            ActiveFlightSettings.drougeChuteEnabled = args[0] == 1;
            ActiveFlightSettings.dualDeploymentEnabled = args[1] == 1;
            // mainDeploymentAltitude is uint16_t, so we need to combine the two bytes
            ActiveFlightSettings.mainDeploymentAltitude = (args[2] << 8) || args[3];

            printlogf("Set flight config: Dual Deploy: %s, Drouge Deploy: %s, Main Deploy Alt: %s\n",
                      ActiveFlightSettings.dualDeploymentEnabled ? "YES" : "NO",
                      ActiveFlightSettings.drougeChuteEnabled ? "YES" : "NO",
                      ActiveFlightSettings.dualDeploymentEnabled ? String(ActiveFlightSettings.mainDeploymentAltitude) : "N/A");
            flightConfigured = true;
            ackNack = true;
        }
        else
        {
            reason = 1;
        }
        break;

    default:
        reason = 4;
        break;
    }

    *responsePacketLength = formCmdAckPacket(responsePacketBuffer, salt, cmd, ackNack, reason);
    return ackNack;
}

void loop()
{
// Not really used as everything is done in tasks
#ifdef AUTO_ARM
    if (!autoArmed)
    {
        if (autoArmCalibratedBy == 0)
        {
            Serial.println("AutoArm enabled: entering calibration");
            stateChange(State::CALIBRATE);
            autoArmCalibratedBy = millis() + 500;
        }
        else if (autoArmCalibratedBy <= millis())
        {
            Serial.println("AutoArm enabled: Calibration finished; Arming");
            stateChange(State::ARMED);
            autoArmed = true;
        }
    }
#endif
    previousTime = currentTime;
    currentTime = millis();
    vTaskDelay(10);
}
#endif