#include "main.h"

void listDir(const char *dirname, uint8_t levels)
{
    printlogf("Listing directory: %s\n", dirname);

    File root = SD.open(dirname);
    if (!root)
    {
        printlnlog("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        printlnlog("Not a directory");
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
        printlnlog("Dir created");
    }
    else
    {
        printlnlog("mkdir failed");
    }
}

void readFile(const char *path)
{
    printlogf("Reading file: %s\n", path);

    File file = SD.open(path);
    if (!file)
    {
        printlnlog("Failed to open file for reading");
        return;
    }

    printlog("Read from file: ");
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
        printlnlog("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        printlnlog("File written");
    }
    else
    {
        printlnlog("Write failed");
    }
    file.close();
}

void appendFile(const char *path, const char *message)
{
    printlogf("Appending to file: %s\n", path);

    File file = SD.open(path, FILE_APPEND);
    if (!file)
    {
        printlnlog("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        printlnlog("Message appended");
    }
    else
    {
        printlnlog("Append failed");
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
    logFile.close();
    eventsFile.close();
    dataFile.close();
    flightFilesReady = false;
}

void initErrorLoop()
{
    digitalWrite(ERROR_LED, HIGH);
    bool toggler = false;
    while (true)
    {
        digitalWrite(STATUS_LED, toggler);
        digitalWrite(BUZZER_PIN, toggler);
        toggler = !toggler;
        delay(500);
    }
}

void allocateHistoryMemory()
{
    printlog("Allocating ");
    printlog(HISTORY_SIZE * LOG_SENSOR_COUNT * sizeof(float));
    printlog(" Bytes for history buffers with: ");
    printlog(LOG_SENSOR_COUNT);
    printlnlog(" sensors");
    printlogf("\n\navailable heap befor allocating %i\n", ESP.getFreeHeap());
    printlogf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    altitudeHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    speedHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    printlogf("\n\navailable heap after allocating %i\n", ESP.getFreeHeap());
    printlogf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
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
        if (useImuOffsets)
        {
            printlnlog("- Setting IMU offsets...");
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
        printlnlog("- Failed to connect to MPU6050!");
        initErrorLoop();
    }
    printlnlog("(BMP):");
    bmp_state = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    if (!bmp_state)
    {
        printlnlog("- Failed to connect to BMP280!");
        initErrorLoop();
    }
    else
    {
        printlog("- Connected to BMP280! SensorID: 0x");
        printlog(bmp.sensorID(), 16);
    }
    printlnlog("- Setting BMP280 parameters...");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    printlnlog("- Done! BMP online");
    printlnlog("Connecting to GPS...");
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    printlnlog("- Done! GPS online");
#endif
}

void pollGps()
{
#ifndef SIM_MODE
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
}

void initWiFi()
{
    printlnlog("Starting AP");
    WiFi.softAP(SSID, psk);
    IPAddress IP = WiFi.softAPIP();
    printlog("- AP IP address: ");
    printlnlog(IP);
}

void initSD()
{
    printlnlog("Init SD card");
    if (!SD.begin(5, SPI, 4000000U, "/sd", 5, true))
    {
        printlnlog("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        printlnlog("No SD card attached");
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

uint16_t readFlightId()
{
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
}

bool saveMetaData()
{
    fs::File file = SD.open("/cflight.yaml", FILE_WRITE);
    if (!file)
    {
        printlnlog("Can't open file for writing");
        return false;
    }
    file.printf("flightId:%i\n", flight_id);
    file.close();
    return true;
}

void setup()
{
    // initialize serial communication
    Serial.begin(115200);
    printlnlog("CFlight v" + String(VERSION));
    pinMode(STATUS_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    // initialize devices
    initSD();
    initSensors();
    initWiFi();
#ifdef RAM_LOG_ENABLED
    allocateHistoryMemory();
#endif
    flight_id = readFlightId() + 1;
    printlnlog("Boot complete!");
    previousTime = millis();
    currentTime = millis();
#ifdef SIM_MODE
    commandCollector = "";
    printlnlog("Sim mode: awaiting packets");
    while (commandCollector != "START")
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

void logData()
{
    if (loggingData)
    {
        if (lastHistoryEntry >= HISTORY_INTERVAL)
        {
#ifdef RAM_LOG_ENABLED
            altitudeHistory[logIndex] = altitude;
            speedHistory[logIndex] = verticalVelocity;
            logIndex++;
            if (logIndex >= HISTORY_SIZE)
            {
                printlnlog("Log full!");
                loggingData = false;
            }
#endif
            dataFile.printf("%i,%f,%f,%f,%f\n", currentTime, altitude, verticalVelocity, bmpTemperature, pressure);
#ifdef FLUSH_EVERY
            flushCounterData++;
            if (flushCounterData >= FLUSH_EVERY)
            {
                dataFile.flush();
            }
#endif
        }
        lastHistoryEntry += deltaTime;
    }
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
    eventsFile.printf("[%s] %s", formatTimestamp(currentTime), s);
    eventsFile.println();
#ifdef FLUSH_EVERY
    flushCounterEvents++;
    if (flushCounterEvents >= FLUSH_EVERY)
    {
        eventsFile.flush();
    }
#endif
}

void stateChange(State newState)
{
    sprintf(fmtBuffer, "State change %s -> %s", StateNames[(int)state], StateNames[(int)newState]);
    printlnlog(fmtBuffer);
    eventEntry(fmtBuffer);
    fmtBuffer[0] = '\0';
    state = newState;
    stateChanged = 2; // State JUST changed
    // Update master arm
    if (newState == State::ARMED)
    {
        saveMetaData();
        createFlightFiles(flight_id);
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
    /*printlnlog("-- Data --");
    printlnlog("time,altitude,speed");
    for (int i = 0; i < finalLogIndex; i++)
    {
        printlog(i * HISTORY_INTERVAL);
        printlog(",");
        printlog(altitudeHistory[i]);
        printlog(",");
        printlog(speedHistory[i]);
        printlnlog();
    }*/
    printlnlog("End of report");
}

void tick()
{
    if (stateChanged != 0)
    {
        stateChanged--;
    }

    if (activeCommand == Command::TOGGLE_SOUND)
    {
        buzzerState = !buzzerState;
    }
    if (state == State::IDLE)
    {
        if (activeCommand == Command::SYSTEMCHECK)
        {
            systemCheck();
        }

        else if (activeCommand == Command::TOGGLE_DROUGE_ENABLED)
        {
            drougeChuteEnabled = !drougeChuteEnabled;
        }
        else if (activeCommand == Command::TOGGLE_DUAL_DEPLOY)
        {
            dualDeploymentEnabled = !dualDeploymentEnabled;
        }
        if (stateChanged == 1 && !calibrate_done)
        {
            initalPresure = pressure; // First tick in cal mode, set initalPressure
        }
        else
        {
            calibrate_done = true;
            // Add current to inital presure & divide by two ("running" avgerage) [TODO: check this]
            initalPresure += pressure;
            initalPresure /= 2;
        }
        if (activeCommand == Command::ARM)
        {
            stateChange(State::ARMED);
        }
        else if (activeCommand == Command::UNARM)
        {
            stateChange(State::IDLE);
        }
    }
    else
    {
        if (state == State::ARMED)
        {
            if (activeCommand == Command::UNARM)
            {
                stateChange(State::IDLE);
            }
            else
            {
                // We are looking for a takeoff event
                if (altitude >= LAUNCH_DETECT_THRESHOLD)
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
                    }
                }
                else
                {
                    launchDetectTicker = 0;
                    loggingData = false;
                    logIndex = 0;
                }
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
            buzzerState = true;
            if (stateChanged == 1 && drougeChuteEnabled)
            {
                // Fire drouge chute
                firePyroCH(0);
            }
            else if (dualDeploymentEnabled && !mainChuteFired && altitude <= mainDeploymentAltitude)
            {
                // Fire dual-deployment main chute
                firePyroCH(1);
            }
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

            if (activeCommand == Command::REPORT || stateChanged == 1)
            {
                loggingData = false;
                finalLogIndex = logIndex;
                eventEntry("Landed, ending flight");
                closeFlightFiles();
                report();
            }
        }
    }
}

void updateOutputs()
{
    if (buzzerState)
    {
        tone(BUZZER_PIN, 2000, 500);
    }
    else
    {
        digitalWrite(BUZZER_PIN, LOW);
    }

    if (currentTime - lastLogEvent >= LOG_INTERVAL)
    {
        lastLogEvent = currentTime;
        minimalLog();
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
        digitalWrite(STATUS_LED, HIGH);
    }
    if (state == State::IDLE)
    {
        if (currentTime - lastLedEvent >= idleLedPattern[ledPatternStage])
        {
            lastLedEvent = currentTime;
            if (ledPatternStage % 2 == 0)
            {
                digitalWrite(STATUS_LED, LOW);
            }
            else
            {
                digitalWrite(STATUS_LED, HIGH);
            }
            ledPatternStage++;
            if (ledPatternStage >= 4)
            {
                ledPatternStage = 0;
                digitalWrite(STATUS_LED, HIGH);
            }
        }
    }
    else if (state == State::CALIBRATE)
    {
        if (lastLedEvent == 0)
        {
            lastLedEvent = currentTime;
            digitalWrite(STATUS_LED, HIGH);
        }
        else if (currentTime - lastLedEvent >= calibrateLedPattern[ledPatternStage])
        {
            lastLedEvent = currentTime;
            if (ledPatternStage == 0)
            {
                ledPatternStage = 1;
                digitalWrite(STATUS_LED, LOW);
            }
            else
            {

                ledPatternStage = 0;
                digitalWrite(STATUS_LED, HIGH);
            }
        }
    }
    else if (state == State::ARMED)
    {
        if (lastLedEvent == 0)
        {
            lastLedEvent = currentTime;
            digitalWrite(STATUS_LED, HIGH);
        }
        else if (currentTime - lastLedEvent >= armedLedPattern[ledPatternStage])
        {
            lastLedEvent = currentTime;
            if (ledPatternStage == 0)
            {
                ledPatternStage = 1;
                digitalWrite(STATUS_LED, LOW);
            }
            else
            {

                ledPatternStage = 0;
                digitalWrite(STATUS_LED, HIGH);
            }
        }
    }
    else if (state == State::GROUNDSTATION)
    {
        if (lastLedEvent == 0)
        {
            lastLedEvent = currentTime;
            digitalWrite(STATUS_LED, HIGH);
        }
        else if (currentTime - lastLedEvent >= groundLedPattern[ledPatternStage])
        {
            lastLedEvent = currentTime;
            if (ledPatternStage % 2 == 0)
            {
                digitalWrite(STATUS_LED, LOW);
            }
            else
            {
                digitalWrite(STATUS_LED, HIGH);
            }
            ledPatternStage++;
            if (ledPatternStage >= 4)
            {
                ledPatternStage = 0;
                digitalWrite(STATUS_LED, HIGH);
            }
        }
    }
}

void saveFlight()
{
    printlnlog("Unimplemented");
}

Command parseCmd()
{
    commandCollector.replace("\n", "");
    printlog("Command: ");
    printlnlog(commandCollector);
#ifndef SIM_MODE

    // 0: Toggle_dual_deployment, so on
    if (commandCollector == "ARM")
        return Command::ARM;
    if (commandCollector == "UNARM")
        return Command::UNARM;
    if (commandCollector == "CALI")
        return Command::ENTER_CALIBRATE;
    if (commandCollector == "BEEP")
        return Command::TOGGLE_SOUND;
    if (commandCollector == "TDUAL")
        return Command::TOGGLE_DUAL_DEPLOY;
    if (commandCollector == "TDROUGE")
        return Command::TOGGLE_DROUGE_ENABLED;
    if (commandCollector == "REPORT")
        return Command::REPORT;
    if (commandCollector == "SCHECK")
        return Command::SYSTEMCHECK;
    if (commandCollector == "LOCATE")
        return Command::LOCATE;
    if (commandCollector == "GNDST")
        return Command::ENTER_GROUNDSTATION;
    if (commandCollector == "SAVE")
    {
        saveFlight();
    }
    if (commandCollector == "RST")
    {
        ESP.restart();
    }
    printlnlog("Unknown command");
    return Command::NONE;
#endif

    return Command::NONE;
}

void readCmd()
{
    if (ground_station.connected())
    {
        handleWifi();
    }
    else
    {
        if (Serial.available() > 0)
        {
#ifndef SIM_MODE
            char c = Serial.read();
            commandCollector += c;
            if (commandCollector.endsWith("\n"))
            {
                activeCommand = parseCmd();
                commandCollector = "";
            }
            else
            {
                activeCommand = Command::NONE;
            }
#endif
#ifdef SIM_MODE

            long currentTimeT = Serial.parseInt();
            if (currentTimeT != 0 && currentTimeT > currentTime)
            {
                currentTime = currentTimeT;

#ifdef DEBUG
                printlog("time: ");
                printlog(currentTime);
#endif

                pressure = Serial.parseFloat();
                if (initalPresure == 0 || initalPresure == nan)
                {
                    initalPresure = pressure;
                }
#ifdef DEBUG
                printlog(" pressure: ");
                printlog(pressure);
#endif
                bmpTemperature = Serial.parseFloat();
#ifdef DEBUG
                printlog(", temp: ");
                printlog(bmpTemperature);
#endif
            }
#endif
        }
        else
        {
            activeCommand = Command::NONE;
        }
    }
}

void handleWifi()
{
    // Is there any data from the ground station?
    if (ground_station.available() > 0)
    {
        String command = ground_station.readStringUntil('\n');
        command.replace("\n", "");
        printlog("Command from ground station: ");
        printlnlog(command);
        // Split command into command and data
        // Format: command:data0,data1,data2,...,dataN;
        int colonIndex = command.indexOf(":");
        if (colonIndex == -1)
        {
            printlnlog("Invalid command from ground station");
            return;
        }
        String commandName = command.substring(0, colonIndex);
        char *commandData[4];
        int dataIndex = 0;
        if (colonIndex + commandName.length() != command.length() - 1)
        {
            int lastCommaIndex = colonIndex;
            for (int i = colonIndex + 1; i < command.length(); i++)
            {
                if (command.charAt(i) == ',')
                {
                    commandData[dataIndex] = (char *)command.substring(lastCommaIndex + 1, i).c_str();
                    dataIndex++;
                    lastCommaIndex = i;
                }
                else if (command.charAt(i) == ';')
                {
                    commandData[dataIndex] = (char *)command.substring(lastCommaIndex + 1, i).c_str();
                    break;
                }
            }
        }
        if (commandName == "arm")
        {
            activeCommand = Command::ARM;
        }
        else if (commandName == "unarm")
        {
            activeCommand = Command::UNARM;
        }
        else if (commandName == "calibrate")
        {
            activeCommand = Command::ENTER_CALIBRATE;
        }
        else if (commandName == "dual_deploy")
        {
            // Data contains dual deploy details (eg dual deploy alt, drouge enabled, etc)
            if (dataIndex != 3)
            {
                printlnlog("Invalid dual deploy command from ground station");
                return;
            }
            dualDeploymentEnabled = commandData[0] == "1";
            drougeChuteEnabled = commandData[1] == "1";
            mainDeploymentAltitude = atoi(commandData[2]);
        }
        else if (commandName == "calibrate")
        {
            activeCommand = Command::ENTER_CALIBRATE;
        }
        else if (commandName == "system_check")
        {
            activeCommand = Command::SYSTEMCHECK;
        }
        else if (commandName == "locate")
        {
            activeCommand = Command::LOCATE;
        }
        else if (commandName == "report")
        {
            activeCommand = Command::REPORT;
        }
        else if (commandName == "beep")
        {
            activeCommand = Command::TOGGLE_SOUND;
        }
    }
}
void minimalLog()
{
    printlogf("[%s] ",
              formatTimestamp(currentTime));
    printlogf("Alt: %fm, ", altitude);
    printlogf("dAlt: %fm/s, ", verticalVelocity);
    printlogf("State: %s\n", StateNames[state]);
}

void loop()
{
#ifndef SIM_MODE
    currentTime = millis();

    deltaTime = currentTime - previousTime;
#endif
    // If we are not connected to a ground station check if there is a new client
    if (wifiServer.hasClient() && !ground_station.connected())
    {
        ground_station = wifiServer.available();
        printlnlog("New client");
        ground_station.println("connected");
    }
    pollImu();
    pollBmp();
    // pollGps();
    readCmd();
#ifdef SIM_MODE
    deltaTime = currentTime - previousTime;
#endif
    tick();
    updateOutputs();
    logData();
    delay(1);
    previousTime = currentTime;
}
