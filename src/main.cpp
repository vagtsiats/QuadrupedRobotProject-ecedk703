#include "definitions.h"
using namespace BLA;
#define TEST
void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    // initial delay before start
    delay(2000);

    Robot.initHardware();
    Robot.drive_legs(conf, conf, conf, conf);
    // Robot.init_walk(1);

    timer0 = micros();
}

void loop()
{

#ifdef TEST
    // Robot.drive_legs(conf, conf, conf, conf);

    // Robot.drive_legs_IK(pos, pos, pos, pos);

#else
    // SECTION - constant time loop :
    if (micros() - timer0 >= LOOP_PERIODus)
    {
        timer0 = micros();

        Robot.walk(micros() / 1.e6, LOOP_PERIODsec);
    }
    ///!SECTION
#endif
}
