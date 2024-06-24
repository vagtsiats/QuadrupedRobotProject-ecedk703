#include "definitions.h"
using namespace BLA;

void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);
    Robot.initHardware();

    Robot.init_trot(10);
    Robot.drive_legs();
    delay(2000);

    timer_0 = micros();
}

void loop()
{

    // constant time loop :
    if (micros() - timer_0 >= LOOP_PERIODus)
    {
        timer_0 = micros();

        Robot.gait(micros() / 1.e6, LOOP_PERIODsec);
    }
}
