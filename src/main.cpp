#include "definitions.h"
using namespace BLA;
void setup()
{
    Serial.begin(9600);
    Robot.initHardware();

    digitalWrite(LED_BUILTIN, 1);

    // NOTE - trajectory test
    //  Trajectory leg_traj(1, 4, -14);
    //  delay(2000);
    //  double period = leg_traj.get_T();

    // for (double t = 0; t < 2 * period; t = t + 0.1)
    // {
    //     Serial.println(fmod(t, period));
    //     printMatrix(leg_traj.get_position(fmod(t, period)));
    // }

    time0 = micros();
}
void loop()
{
    t = micros() - time0; // in micros
    float t_sec = t / 1e6;
    Robot.br.inverseDiffKinematics(theta0, xd(t_sec), xd_dot(t_sec));
    delay(20);
}
