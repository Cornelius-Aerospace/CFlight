#ifndef COMMON_H
#define COMMON_H
#include <const.h>
#include <settings.h>
#include <Arduino.h>
extern SystemSettings SysSettings;
extern FlightSettings ActiveFlightSettings;
void initErrorLoop();
#endif