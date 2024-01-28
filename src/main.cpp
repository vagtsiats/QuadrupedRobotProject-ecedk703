#include "definitions.h"
using namespace BLA;

Trajectory leg_traj(1, 4, -14);

void setup()
{
    Serial.begin(9600);
    Robot.initHardware();

    digitalWrite(LED_BUILTIN, 1);

    // NOTE - trajectory test
    // Trajectory leg_traj(1, 4, -14);
    // delay(2000);
    // double period = leg_traj.get_T();

    // for (double t = 0; t < 2 * period; t = t + 0.1)
    // {
    //     BLAprintMatrix(leg_traj.get_position(fmod(t, period)));
    // }
    time0 = micros();

}
void loop()
{
    t = micros() - time0; // in micros
    float t_sec = t / 1e6;
    double period = leg_traj.get_T();
    Matrix<3,1,double> doubleMatrix1=~leg_traj.get_position(fmod(t_sec, period));
    Matrix<3,1,double> doubleMatrix2= ~leg_traj.get_velocity(fmod(t_sec, period));
    Matrix<3> xd,xd_dot;

    xd(0)=-doubleMatrix1(2);
    xd(1)=doubleMatrix1(1);
    xd(2)=doubleMatrix1(0);

    xd_dot(0) = -doubleMatrix2(2);
    xd_dot(1) = doubleMatrix2(1);
    xd_dot(2) = doubleMatrix2(0);

    Robot.br.inverseDiffKinematics(theta0, xd,xd_dot);


    delay(20);
}
