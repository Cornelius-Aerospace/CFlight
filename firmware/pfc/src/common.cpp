#include "common.h"

void initErrorLoop(SystemSettings settings)
{
    digitalWrite(SysSettings.errorLedPin, HIGH);
    bool toggler = false;
    while (true)
    {
        digitalWrite(SysSettings.statusLedPin, toggler);
        digitalWrite(SysSettings.buzzerPin, toggler);
        toggler = !toggler;
        delay(500);
    }
}
