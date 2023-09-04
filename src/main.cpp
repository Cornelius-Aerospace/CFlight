#include "main.h"

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
    Serial.print("Allocating ");
    Serial.print(HISTORY_SIZE * sizeof(float) * 2);
    Serial.println(" Bytes for history buffers");
    Serial.printf("\n\navailable heap befor allocating %i\n", ESP.getFreeHeap());
    Serial.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    altitudeHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    speedHistory = (float *)calloc(HISTORY_SIZE, sizeof(float));
    Serial.printf("\n\navailable heap after allocating %i\n", ESP.getFreeHeap());
    Serial.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

void initSensors()
{
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
        initErrorLoop();
    }
    Serial.println("(BMP):");
    bmp_state = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    if (!bmp_state)
    {
        Serial.println("- Failed to connect to BMP280!");
        initErrorLoop();
    }
    else
    {
        Serial.print("- Connected to BMP280! SensorID: 0x");
        Serial.print(bmp.sensorID(), 16);
    }
    Serial.println("- Setting BMP280 parameters...");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    Serial.println("- Done! BMP online");
    Serial.println("Connecting to GPS...");
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("- Done! GPS online");
}

void pollGps()
{
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
        Serial.println("No GPS detected: check wiring.");
    }
}

void initWiFi()
{
    Serial.println("Starting AP");
    WiFi.softAP(SSID, psk);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("- AP IP address: ");
    Serial.println(IP);
}

void setup()
{
    // initialize serial communication
    Serial.begin(115200);
    Serial.println("CFlight v" + String(VERSION));
    pinMode(STATUS_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    // initialize devices
    initSensors();
    initWiFi();
    allocateHistoryMemory();
    Serial.println("Boot complete!");
    previousTime = millis();
    currentTime = millis();
}

void logData()
{
    if (loggingData)
    {
        if (lastHistoryEntry >= HISTORY_INTERVAL)
        {
            altitudeHistory[logIndex] = altitude;
            speedHistory[logIndex] = verticalVelocity;
            logIndex++;
            if (logIndex >= HISTORY_SIZE)
            {
                Serial.println("Log full!");
                loggingData = false;
            }
        }
        lastHistoryEntry += deltaTime;
    }
}

void pollImu()
{

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void pollBmp()
{

    pressure = bmp.readPressure();          // SI units (pascal)
    pressure /= 100;                        // Now in MPa
    bmpTemperature = bmp.readTemperature(); // Deg c
    speedTicker += deltaTime;
    altitude = 44330 * (1.0 - pow(pressure / initalPresure, 0.1903)); // meters
    // Calculate vertical speed
    if (speedTicker >= SPEED_INTERVAL)
    {
        // SPEED_INTERVAL has passed, calculate vertical speed
        speedTicker = 0;
        verticalVelocity = (altitude - previousAltitude) / (SPEED_INTERVAL / 1000.0); // m/s
        previousAltitude = altitude;
        if (verticalVelocity > peakVerticalVelocity)
        {
            peakVerticalVelocity = verticalVelocity;
            peakVerticalVelocityTime = currentTime;
        }
    }
    speedTicker += deltaTime;
}

bool firePyroCH(uint8_t channel)
{
    if (!masterArm)
    {
        Serial.print("Refusing to fire pyro CH ");
        Serial.print(channel);
        Serial.println(", not armed!");
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
    Serial.print("Pyro CH ");
    Serial.print(channel);
    Serial.println(" fired");
    return true;
}

void stateChange(State newState)
{
    Serial.print("State change ");
    Serial.print(StateNames[(int)state]);
    Serial.print(" -> ");
    Serial.println(StateNames[(int)newState]);
    state = newState;
    stateChanged = 2; // State JUST changed
    // Update master arm
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
    Serial.println("Begining systems check:");
    // TODO systems check
}

void humanLogTimestamp(unsigned long timestamp)
{
    auto now = timestamp;
    byte minute = (now / 60000);
    now -= (minute * 60000);
    byte second = (now / 1000);
    Serial.print(minute);
    Serial.print(":");
    if (second < 10)
    {
        Serial.print("0");
    }
    Serial.print(second);
}

void report()
{
    Serial.println("Flight Report: ");
    Serial.print("Lift-off at: ");
    humanLogTimestamp(launchEventTimestamp);
    Serial.println();
    Serial.print("Apogee: ");
    Serial.print(peakAltitude);
    Serial.print("m, At time: ");
    humanLogTimestamp(apoggeeEventTimestamp);
    Serial.println();
    Serial.println("Max speed: ");
    Serial.print(peakVerticalVelocity);
    Serial.print("m/s, At time: ");
    humanLogTimestamp(currentTime);
    Serial.print("Touchdown at: ");
    humanLogTimestamp(landingEventTimestamp);
    Serial.println();
    Serial.print("Flight time: ");
    humanLogTimestamp(landingEventTimestamp - launchEventTimestamp);
    Serial.println("-- Data --");
    Serial.print("time,altitude,speed");
    for (int i = 0; i < finalLogIndex; i++)
    {
        Serial.print(i * HISTORY_INTERVAL);
        Serial.print(",");
        Serial.print(altitudeHistory[i]);
        Serial.print(",");
        Serial.print(speedHistory[i]);
        Serial.println();
    }
    Serial.println("End of report");
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
        if (activeCommand == Command::ENTER_CALIBRATE)
        {
            stateChange(State::CALIBRATE);
        }
        else if (activeCommand == Command::SYSTEMCHECK)
        {
            systemCheck();
        }
        else if (activeCommand == Command::ARM && calibrate_done)
        {
            stateChange(State::ARMED);
        }
        else if (activeCommand == Command::TOGGLE_DROUGE_ENABLED)
        {
            drougeChuteEnabled = !drougeChuteEnabled;
        }
        else if (activeCommand == Command::TOGGLE_DUAL_DEPLOY)
        {
            dualDeploymentEnabled = !dualDeploymentEnabled;
        }
    }
    if (state == State::CALIBRATE)
    {
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
                if (verticalVelocity >= LAUNCH_DETECT_THRESHOLD)
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
            if (verticalVelocity > 0 || altitude > peakAltitude)
            {
                peakAltitude = altitude;
                descentTicker = 0;
            }
            else if (verticalVelocity <= LAUNCH_DETECT_THRESHOLD)
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
                report();
            }
        }
    }
}

void updateOutputs()
{
    if (buzzerState)
    {
        digitalWrite(BUZZER_PIN, HIGH);
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

Command parseCmd()
{
    commandCollector.replace("\n", "");
    Serial.print("Command: ");
    Serial.println(commandCollector);
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
    Serial.println("Unknown command");
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
        Serial.print("Command from ground station: ");
        Serial.println(command);
        // Split command into command and data
        // Format: command:data0,data1,data2,...,dataN;
        int colonIndex = command.indexOf(":");
        if (colonIndex == -1)
        {
            Serial.println("Invalid command from ground station");
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
                Serial.println("Invalid dual deploy command from ground station");
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
    // Minimal log over serial
    Serial.print("[");
    Serial.print(currentTime);
    Serial.print("] Alt: ");
    Serial.print(altitude);
    Serial.print(", dAlt: ");
    Serial.print(verticalVelocity);
    Serial.print("m/s, Pressure: ");
    Serial.print(pressure);
    Serial.print(", Inital pressure: ");
    Serial.print(initalPresure);
    Serial.print(", gpsFix: ");
    Serial.print(gpsFix ? "true" : "false");
    Serial.print(", GPS satillites: ");
    Serial.print(gpsSatilliteCount);
    Serial.print(", lat: ");
    Serial.print(gpsLatitude);
    Serial.print(", long: ");
    Serial.print(gpsLongitude);
    Serial.print(", GPS alt: ");
    Serial.print(gpsAltitude);
    Serial.print(", GPS speed: ");
    Serial.print(gpsSpeed);
    Serial.print(", GPS HDOP: ");
    Serial.print(gpsHdop);
    Serial.print(", State: ");
    Serial.println(StateNames[(int)state]);
}

void loop()
{
    previousTime = currentTime;
    currentTime = millis();
    deltaTime = currentTime - previousTime;
    // If we are not connected to a ground station check if there is a new client
    if (wifiServer.hasClient() && !ground_station.connected())
    {
        ground_station = wifiServer.available();
        Serial.println("New client");
        ground_station.println("connected");
    }
    pollImu();
    pollBmp();
    pollGps();
    readCmd();
    tick();
    updateOutputs();
    logData();
    delay(1);
}