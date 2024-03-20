#include "definitions.h"
using namespace BLA;

Trajectory leg_traj(4, -14);
Turn turning(4,-14);
void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    // NOTE - trajectory test
    // Trajectory leg_traj(1, 4, -14);
    // delay(2000);
    // double period = leg_traj.get_T();

    // for (double t = 0; t < 2 * period; t = t + 0.1)
    // {
    //     BLAprintMatrix(leg_traj.get_position(fmod(t, period)));
    // }
    Robot.initHardware();
    turning.calculateTurnParameters();

    timer0 = micros();
}

void loop()
{
    // SECTION - Alex Code
    float t = micros() - timer0; // in micros
     float t_sec = t / 1e6;
     double period = leg_traj.get_T();
     Matrix<3> doubleMatrix1=leg_traj.get_position(fmod(t_sec, period));
     Matrix<3> doubleMatrix2= leg_traj.get_velocity(fmod(t_sec, period));
     Matrix<3> xd,xd_dot;

    xd(0)=-doubleMatrix1(2);
    xd(1)=doubleMatrix1(1);
    xd(2)=doubleMatrix1(0);

    xd_dot(0) = -doubleMatrix2(2);
    xd_dot(1) = doubleMatrix2(1);
    xd_dot(2) = doubleMatrix2(0);

    Matrix<3> theta0=Robot.fr.InverseKinematics({14,0,2});
    // BLAprintMatrix(xd);
    Robot.fr.updateTranslations(theta0);
    Robot.fr.computeJacobian();
    Robot.fr.inverseDiffKinematics(theta0, xd,xd_dot);
    delay(20);

    //! SECTION


    // // SECTION - constant  time loop:
    // if (micros() - timer0 >= LOOP_PERIOD)
    // {
    //     timer0 = micros();
    //     timer1 = timer0 / 1e6;

    //     Robot.gait(timer1);
    //     // NOTE - DiffKinematics Test
    //     // Robot.br.inverseDiffKinematics(theta0, xd(timer1), xd_dot(timer1));
    // }
}
