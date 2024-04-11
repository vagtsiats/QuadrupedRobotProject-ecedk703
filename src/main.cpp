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

    timer0 = micros();
}

void loop()
{
    uint32_t timer1 = micros(); // in micros
    float t_sec = timer1 / 1e6;

    float leg_sample_time = fmod(t_sec, testtraj.get_T());

    Matrix<3> x_d = testtraj.get_position(leg_sample_time);
    Matrix<3> xd_d = testtraj.get_velocity(leg_sample_time);

    // SECTION - constant time loop :
    if (micros() - timer0 >= LOOP_PERIODms)
    {
        timer0 = micros();

        if (init_error(1) > min_error && init_error(1) > min_error && init_error(2) > min_error)
        {
            init_error = testleg.JTranspIK(testtraj.get_position(0), BLAdiagonal<3>(1), LOOP_PERIODsec);
        }
        else
        {
            testleg.JInvIK(x_d, xd_d, BLAdiagonal<3>(5), LOOP_PERIODsec, testtraj.get_position(0));
        }

        // BLAprintMatrix(testleg.getEndEffectorPosition());
        // BLAprintMatrix(error);
        // BLAprintMatrix(testleg.getTheta());s
        // Serial.println();
    }

    // testleg.setTheta({0, 0, 0});

    testleg.DriveLeg();
    testleg.update_leg(testleg.getTheta()); // a bit odd. Check definition of update_leg. Generally define the  "theta" pipeline
}
