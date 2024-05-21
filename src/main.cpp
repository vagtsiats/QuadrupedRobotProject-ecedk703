#include "definitions.h"
using namespace BLA;
// #define TEST
void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    // initial delay before start
    // delay(2000);

    Robot.initHardware();
    // Robot.drive_legs(conf, {0,0,0}, conf, conf);
    // Robot.fr.InverseKinematics({0, -5, -20});
    // Robot.fl.InverseKinematics({0, 5, -20});
    // Robot.bl.InverseKinematics({0, 5, -20});
    // Robot.br.InverseKinematics({0, -5, -20});

    Robot.init_trot(3);

    Robot.drive_legs();
    // Robot.init_walk(1);
    // BLAprintMatrix(Robot.fr.getEndEffectorPosition());
    delay(2000);

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
        // Serial.println(timer0 / 1.e6);

        Robot.walk(micros() / 1.e6, LOOP_PERIODsec);
    }
    ///!SECTION
#endif
}
