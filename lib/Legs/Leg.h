#pragma once

#include "Servo.h"
#include <BasicLinearAlgebra.h>
#include <iostream>
#include <vector>
#include "BLA_tools.h"
class Leg
{
public:
    Leg(int t_pin_shoulder, int t_pin_knee, int t_pin_ankle, const std::vector<float> &t_zeros, const std::vector<float> &t_polar);
    ~Leg();

    void setDh(BLA::Matrix<3> t_dh_a, BLA::Matrix<3> t_dh_alpha, BLA::Matrix<3> t_dh_d);

    void setBodyT(BLA::Matrix<4, 4> t_body_T);

    void DriveLeg(int up, int mid, int low);

    BLA::Matrix<3> getDh(int a);

    BLA::Matrix<4, 4> dhTransform(float a, float alpha, float d, float theta);

    void updateTranslations(BLA::Matrix<3> theta);

    BLA::Matrix<3> forwardKinematics();

    void computeJacobian();

    BLA::Matrix<6, 3> getJacobian();

    BLA::Matrix<3, 3> getJacobianPos();

    void inverseDiffKinematics(BLA::Matrix<3> theta0, BLA::Matrix<3> xd, BLA::Matrix<3> xd_dot);

    /// @brief Inverse Kinematics Algorithm with Jacobian Transpose
    /// @param x_d desired position {x,y,z}
    /// @param Gain 3by3 diagonal
    /// @param dt
    /// @param initial_configuration default = {0,0,0}
    void JTransIK(BLA::Matrix<3> x_d, BLA::Matrix<3, 3> Gain, float dt, BLA::Matrix<3> initial_configuration = {0, 0, 0});

    void resetInitialPos();

    void attach_servos();
    BLA::Matrix<3> InverseKinematics(BLA::Matrix<3> pos);

private:
    // Servos
    Servo shoulder;
    Servo knee;
    Servo ankle;
    // Zero Vectors and Polarity Vector
    // Zero is where the angle of each servo coresponds to 0 in the DH params
    // Polarity is +/- 1 such that the servos rotate in the same dir as the DH Frames
    float zeros[3];
    float polar[3];
    // Pins
    int pin_shoulder;
    int pin_knee;
    int pin_ankle;
    // Position Relative to body
    BLA::Matrix<4, 4> body_T;
    // DH
    BLA::Matrix<3> dh_alpha;
    BLA::Matrix<3> dh_a;
    BLA::Matrix<3> dh_d;
    // Position of End effector in frame-0
    BLA::Matrix<3> pe;
    // Other link Position vectors
    BLA::Matrix<3> p1;
    BLA::Matrix<3> p2;

    BLA::Matrix<4, 4> BaseFrameTranslation;

    // Translation between frames.
    BLA::Matrix<4, 4> T01;
    BLA::Matrix<4, 4> T12;
    BLA::Matrix<4, 4> T23;
    BLA::Matrix<4, 4> T02;
    BLA::Matrix<4, 4> T03;

    BLA::Matrix<3> z0;
    BLA::Matrix<3> z1;
    BLA::Matrix<3> z2;

    // Jacobians for Position, Orientation and total
    BLA::Matrix<3, 3> Jp;
    BLA::Matrix<3, 3> Jo;
    BLA::Matrix<6, 3> Jacobian;

    bool initialisation = true;
    BLA::Matrix<3> theta;
};
