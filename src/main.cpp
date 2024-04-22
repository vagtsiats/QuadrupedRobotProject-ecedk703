#include "definitions.h"
using namespace BLA;

void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    BLA::Matrix<3> fl_dh_a = {3, 7, 5}; // in CM
    BLA::Matrix<3> fl_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> fl_dh_d = {0, 0, 0};
    testleg.setDh(fl_dh_a, fl_dh_alpha, fl_dh_d);

    testleg.attach_servos();

    testleg.DriveLeg({0, 0, 0});
    delay(1000);

    timer0 = millis();
}

void loop()
{
    timer1 = millis();

    // SECTION - constant time loop :
    if (millis() - timer0 >= LOOP_PERIODms)
    {
        timer0 = millis();

        Matrix<3> x_d = testtraj.get_position(timer1 / 1e3);
        Matrix<3> xd_d = testtraj.get_velocity(timer1 / 1e3);

        if ((abs(init_error(1)) > min_error) || (abs(init_error(1)) > min_error) || (abs(init_error(2)) > min_error))
        {
            init_error = testleg.JTranspIK(testtraj.get_position(0), BLAdiagonal<3>(0.01), LOOP_PERIODsec);
            timer2 = millis();
            BLAprintMatrix(init_error);
        }
        else if (initialization)
        {
            if (millis() - timer2 > 2e3)
            {
                Serial.println("Init_done");
                initialization = false;
            }
        }

        if (!initialization)
        {
            testleg.JInvIK(x_d, xd_d, BLAdiagonal<3>(5), LOOP_PERIODsec);
        }

        // BLAprintMatrix(testleg.getEndEffectorPosition());
        // BLAprintMatrix(testleg.getTheta());

        // Serial.println();
    }
    //! SECTION

    testleg.DriveLeg();
    testleg.update_leg(testleg.getTheta()); // a bit odd. Check definition of update_leg. Generally define the  "theta" pipeline
}
