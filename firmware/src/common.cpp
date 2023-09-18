#include "common.h"

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
