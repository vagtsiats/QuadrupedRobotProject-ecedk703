#include "definitions.h"
using namespace BLA;

Trajectory leg_traj(4, -14);
Turn turning(4, -14);

void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    Robot.initHardware();

    // turning.calculateTurnParameters();

    timer0 = micros();
}

void loop()
{
    // SECTION - Alex Code
    float t = micros() - timer0; // in micros
    float t_sec = t / 1e6;
    double period = leg_traj.get_T();
    Matrix<3> xd=leg_traj.get_position(fmod(t_sec, period));
    Matrix<3> xd_dot= leg_traj.get_velocity(fmod(t_sec, period));

    Matrix<3> theta0=Robot.br.InverseKinematics({14,0,2});
    // BLAprintMatrix(xd);

    Robot.br.inverseDiffKinematics(theta0,xd,xd_dot);
    
    delay(150);

}
