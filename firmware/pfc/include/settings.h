#ifndef SETTINGS_H
#define SETTINGS_H
#include <Arduino.h>

class SystemSettings
{
    // A helper class to store system settings
public:
    uint8_t useImuOffsets;
    uint8_t axOffset;
    uint8_t ayOffset;
    uint8_t azOffset;
    uint8_t gxOffset;
    uint8_t gyOffset;
    uint8_t gzOffset;

    unsigned long master_code;
    unsigned long temp_arm_code;

    int gpsRxPin;
    int gpsTxPin;
    int statusLedPin;
    int errorLedPin;
    int buzzerPin;

    float launchDetectThreshold;
    uint launchDetectTicks;
    float descentDetectThreshold;
    uint descentDetectTicks;
    float landedDetectThresholdLow;
    float landedDetectThresholdHigh;
    uint landedDetectTicks;

    unsigned long logInterval;
    unsigned long speedInterval;

    SystemSettings()
    { // Default
        useImuOffsets = false;
        axOffset = 0;
        ayOffset = 0;
        azOffset = 0;
        gxOffset = 0;
        gyOffset = 0;
        gzOffset = 0;

        master_code = 202910;
        temp_arm_code = 12345;

        gpsRxPin = 16;
        gpsTxPin = 17;
        statusLedPin = 2;
        errorLedPin = 12;
        buzzerPin = 13;

        launchDetectThreshold = 1.0;
        launchDetectTicks = 10;
        descentDetectThreshold = 1.0;
        descentDetectTicks = 10;
        landedDetectThresholdLow = -0.4;
        landedDetectThresholdHigh = 0.4;
        landedDetectTicks = 100;

        logInterval = 1000;
        speedInterval = 50;
    }
};

class FlightSettings
{
public:
    unsigned long mainDeploymentAltitude;     // Altitude at which main chute deployed (0 to disable)
    bool drougeChuteEnabled;                  // If true fire DrougeCH pyro at detect apogee + delay
    bool dualDeploymentEnabled;               // If true wait for mainDeploymentAltitude and then fire DDMainCH
    unsigned long drougeChuteDeploymentDelay; // Delay (millis) after apogee to fire drouge

    FlightSettings(unsigned long mainDeploymentAltitude, bool drougeChuteEnabled, bool dualDeploymentEnabled, unsigned long drougeChuteDeploymentDelay)
    {
        this->mainDeploymentAltitude = mainDeploymentAltitude;
        this->drougeChuteEnabled = drougeChuteEnabled;
        this->dualDeploymentEnabled = dualDeploymentEnabled;
        this->drougeChuteDeploymentDelay = drougeChuteDeploymentDelay;
    }
    
    FlightSettings() { // Default settings
        this->mainDeploymentAltitude = 200;
        this->drougeChuteEnabled = false;
        this->dualDeploymentEnabled = false;
        this->drougeChuteDeploymentDelay = false;
    }
};

SystemSettings SysSettings = SystemSettings();
FlightSettings ActiveFlightSettings = FlightSettings();
#endif