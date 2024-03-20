#include "Leg.h"
using namespace BLA;
Leg::Leg(int t_pin_shoulder, int t_pin_knee, int t_pin_ankle, const std::vector<float> &t_zeros, const std::vector<float> &t_polar)
{
    pin_shoulder = t_pin_shoulder;
    pin_knee = t_pin_knee;
    pin_ankle = t_pin_ankle;

    theta = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        zeros[i] = t_zeros[i];
        polar[i] = t_polar[i];
    }

    BaseFrameTranslation = {0, 0, 1, 0,
                            0, 1, 0, 0,
                            -1, 0, 0, 0,
                            0, 0, 0, 1};
}

Leg::~Leg()
{
}
void Leg::attach_servos()
{
    shoulder.attach(pin_shoulder);
    knee.attach(pin_knee);
    ankle.attach(pin_ankle);
}

void Leg::setDh(BLA::Matrix<3> t_dh_a, BLA::Matrix<3> t_dh_alpha, BLA::Matrix<3> t_dh_d)
{
    dh_a = t_dh_a;
    dh_alpha = t_dh_alpha;
    dh_d = t_dh_d;
}
void Leg::DriveLeg(int up, int mid, int low)
{

    shoulder.write(polar[0] * (rad2deg(up) - zeros[0]));
    knee.write(polar[1] * (rad2deg(mid) - zeros[1]));
    ankle.write(polar[2] * (rad2deg(low) - zeros[2]));
}
BLA::Matrix<3> Leg::getDh(int a)
{
    if (a == 0)
    {
        return dh_a;
    }
    else if (a == 1)
    {
        return dh_alpha;
    }
    else
    {
        return dh_d;
    }
}

Matrix<4, 4> Leg::dhTransform(float a, float alpha, float d, float theta)
{
    Matrix<4, 4> T = {cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
                      sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                      0, sin(alpha), cos(alpha), d,
                      0, 0, 0, 1};
    return T;
}

void Leg::updateTranslations(BLA::Matrix<3> theta)
{
    T01 = BaseFrameTranslation * dhTransform(dh_a(0), dh_alpha(0), dh_d(0), theta(0));
    T12 = dhTransform(dh_a(1), dh_alpha(1), dh_d(1), theta(1));
    T23 = dhTransform(dh_a(2), dh_alpha(2), dh_d(2), theta(2));
    T02 = T01 * T12;
    T03 = T02 * T23;

    pe = {T03(0, 3), T03(1, 3), T03(2, 3)};
    return;
}

Matrix<3> Leg::forwardKinematics()
{
    return pe;
}

void Leg::computeJacobian()
{
    z0 = {BaseFrameTranslation(0, 2), BaseFrameTranslation(1, 2), BaseFrameTranslation(2, 2)};
    z1 = {T01(0, 2), T01(1, 2), T01(2, 2)};
    z2 = {T02(0, 2), T02(1, 2), T02(2, 2)};
    p1 = {T01(0, 3), T01(1, 3), T01(2, 3)};
    p2 = {T02(0, 3), T02(1, 3), T02(2, 3)};

    Jo = z0 || z1 || z2;

    Jp = crossProduct(z0, pe) || crossProduct(z1, pe - p1) || crossProduct(z2, pe - p2);

    Jacobian = Jo && Jp;
    return;
}

Matrix<6, 3> Leg::getJacobian()
{

    return Jacobian;
}

Matrix<3, 3> Leg::getJacobianPos()
{

    return Jp;
}

void Leg::inverseDiffKinematics(Matrix<3> theta0, Matrix<3> xd, Matrix<3> xd_dot)
{

    Matrix<3, 3> K = {1, 0, 0,
                      0, 1, 0,
                      0, 0, 1};
    float dt = 0.02;

    if (initialisation)
    {
        theta = theta0;
        initialisation = false;
    }

    updateTranslations(theta);
    computeJacobian();

    // For theta 0 -pi/4 -pi/4 -> 7.95 0 -9.95 so we want to move on the z axis only with inverse dif kinematics:
    Matrix<3> error = xd - forwardKinematics();
    theta += (Inverse(getJacobianPos()) * (xd_dot + K * error)) * dt;
    BLAprintMatrix(forwardKinematics());
    // BLAprintMatrix(error);
}

void Leg::JTransIK(BLA::Matrix<3> x_d, BLA::Matrix<3, 3> Gain, float dt, BLA::Matrix<3> initial_configuration)
{

    if (initialisation)
    {
        theta = initial_configuration;
        initialisation = false;
    }

    updateTranslations(theta);
    computeJacobian();

    // For theta 0 -pi/4 -pi/4 -> 7.95 0 -9.95 so we want to move on the z axis only with inverse dif kinematics:
    Matrix<3> error = x_d - forwardKinematics();

    theta += (~(getJacobianPos()) * (Gain * error)) * dt;

    BLAprintMatrix(forwardKinematics());
    // BLAprintMatrix(error);
}

void Leg::resetInitialPos()
{
    initialisation = false;
    return;
}

Matrix<3> Leg::InverseKinematics(Matrix<3> pos)
{
    float x = pos(0);
    float y = pos(1);
    float z = pos(2);
    Matrix<3> theta;
    theta(0) = atan(y / x);
    float c1 = cos(theta(0));

    float R = sqrt(pow(x / c1 - dh_a(0), 2) + pow(z, 2));
    float R2 = acos((pow(dh_a(1), 2) + pow(R, 2) - pow(dh_a(2), 2)) / (2 * dh_a(1) * R));
    float R3 = acos((pow(dh_a(1), 2) + pow(dh_a(2), 2) - pow(R, 2)) / (2 * dh_a(1) * dh_a(2)));
    float b2 = atan(z / (x / c1 - dh_a(0)));

    theta(1) = -R2 + b2;

    theta(2) = M_PI - R3;

    return theta;
}