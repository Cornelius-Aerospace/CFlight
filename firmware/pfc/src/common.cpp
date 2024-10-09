#include "common.h"

void initErrorLoop(SystemSettings settings)
{
    digitalWrite(settings.errorLedPin, HIGH);
    bool toggler = false;
    while (true)
    {
        digitalWrite(settings.statusLedPin, toggler);
        digitalWrite(settings.buzzerPin, toggler);
        toggler = !toggler;
        delay(500);
    }
}
