#include "definitions.h"
using namespace BLA;

void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    Robot.initHardware();

    timer0 = micros();
}

void loop()
{
}
