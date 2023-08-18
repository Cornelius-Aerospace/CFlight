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
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

    Serial.println("- Done! BMP online");
}

void setup()
{
    // initialize serial communication
    Serial.begin(115200);
    Serial.println("CFlight v" + String(VERSION));
    pinMode(STATUS_LED, OUTPUT);
    // initialize devices
    initSensors();
    Serial.println("Boot complete!");
    previousTime = millis();
}

void logData()
{
    // TODO
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

void report() {
    Serial.println("Flight Report: ");
    Serial.print("Lift-off at: ");
    Serial.print(launchEventTimestamp);
    Serial.println("ms");
    Serial.print("Apogee: ");
    Serial.print(peakAltitude);
    Serial.print("m, At time: ");
    Serial.print(apoggeeEventTimestamp);
    Serial.println("ms");
    Serial.print("Touchdown at: ");
    Serial.print(landingEventTimestamp);
    Serial.println("ms");
    Serial.print("Flight time: ");
    Serial.print(landingEventTimestamp - launchEventTimestamp);
    Serial.println("ms");
}

void tick()
{
    if (stateChanged != 0)
    {
        stateChanged--;
    }
    altitude = 44330 * (1.0 - pow(pressure / initalPresure, 0.1903)); // meters
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
    }
    if (state == State::CALIBRATE)
    {
        if (stateChanged == 1)
        {
            initalPresure = pressure; // First tick in cal mode, set initalPressure
        }
        else
        {
            // Add current to inital presure & divide by two ("running" avgerage) [TODO: check this]
            initalPresure += pressure;
            initalPresure /= 2;
        }
        if (activeCommand == Command::ARM)
        {
            stateChange(State::ARMED);
        }
    }
    else
    {
        // Altitude from pressure
        previousAltitude = altitude;
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
                }
            }
        }
        else if (state == State::ASCENT)
        {
            // Are we still in-ascent
            deltaAltitude = altitude - previousAltitude;
            if (deltaAltitude > 0 || altitude > peakAltitude)
            {
                peakAltitude = altitude;
                descentTicker = 0;
            }
            else if (deltaAltitude <= LAUNCH_DETECT_THRESHOLD)
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
            if (deltaAltitude >= LANDED_DETECT_THRESHOLD_LOW && deltaAltitude <= LANDED_DETECT_THRESHOLD_HIGH)
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
        } else if (state == State::LANDED) {
            if (activeCommand == Command::REPORT) {
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
    if (stateChanged == 1) {
        ledPatternStage = 0;
    }
    if (state == State::IDLE)
    {
        if (lastLedEvent == 0)
        {
            lastLedEvent = currentTime;
            digitalWrite(STATUS_LED, HIGH);
        }
        else if (currentTime - lastLedEvent >= idleLedPattern[ledPatternStage])
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
            if (ledPatternStage >= 3)
            {
                ledPatternStage == 0;
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

                ledPatternStage == 0;
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

                ledPatternStage == 0;
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
            if (ledPatternStage >= 3)
            {
                ledPatternStage == 0;
                digitalWrite(STATUS_LED, HIGH);
            }
        }
    }
}

void readCmd()
{
    if (Serial.available() > 0)
    {
        activeCommand = static_cast<Command>(Serial.read());
        Serial.println((int) activeCommand);
    }
    else
    {
        activeCommand = Command::NONE;
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
    Serial.print(deltaAltitude);
    Serial.print(", Pressure: ");
    Serial.print(pressure);
    Serial.print(", Inital pressure: ");
    Serial.print(initalPresure);
    Serial.print(", ledPatternStage: ");
    Serial.print(ledPatternStage);
    Serial.print(", lastLedEvent: ");
    Serial.print(lastLedEvent);
    Serial.print(", State: ");
    Serial.println(StateNames[(int)state]);
}

void loop()
{
    currentTime = millis();
    pollImu();
    pollBmp();
    readCmd();
    tick();
    updateOutputs();
    delay(1);
}