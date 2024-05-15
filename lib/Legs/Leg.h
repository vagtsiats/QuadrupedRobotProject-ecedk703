#pragma once

#include "Servo.h"
#include <BasicLinearAlgebra.h>
#include <iostream>
#include <vector>
#include "BLA_tools.h"
class Leg
{
public:
    /// @brief
    /// @param t_pin_shoulder
    /// @param t_pin_knee
    /// @param t_pin_ankle
    /// @param t_zeros
    /// @param t_polar
    Leg(int t_pin_shoulder, int t_pin_knee, int t_pin_ankle, const std::vector<float> &t_zeros, const std::vector<float> &t_polar);

    void setDh(BLA::Matrix<3> t_dh_a, BLA::Matrix<3> t_dh_alpha, BLA::Matrix<3> t_dh_d);

    void setBodyT(BLA::Matrix<4, 4> t_body_T);

    /// @brief implicitly drive leg servos to computed angles
    void driveLeg();

    /// @brief explicitly drive leg servos to given angles
    /// @param desired angles
    void driveLeg(BLA::Matrix<3> th);

    BLA::Matrix<3> getDh(int a);

    void attach_servos();

    const BLA::Matrix<3> getEndEffectorPosition();

    const BLA::Matrix<6, 3> getJacobian();

    const BLA::Matrix<3, 3> getJacobianPos();

    /// @return servo positions in radians {shoulder, knee, ankle}
    const BLA::Matrix<3> getTheta();

    void setTheta(BLA::Matrix<3> t_theta);

    /// @brief Inverse Kinematics Algorithm with Jacobian Inverse. Used for following a desired trajectory
    /// @param x_des desired position {x,y,z}
    /// @param xd_des desired velocity {vx,vy,vz}
    /// @param t_gain 3by3 diagonal gain matrix
    /// @param t_dt
    /// @param t_initial_configuration
    /// @returns the error
    const BLA::Matrix<3> JInvIK(BLA::Matrix<3> x_des, BLA::Matrix<3> xd_des, BLA::Matrix<3, 3> t_gain, float t_dt, BLA::Matrix<3> initial_configuration = {0, 0, 0});

    /// @brief Inverse Kinematics Algorithm with Jacobian Transpose. Used for initializing the leg position (maybe removed)?
    /// @param x_d desired position {x,y,z}
    /// @param t_gain 3by3 diagonal gain matrix
    /// @param t_dt
    /// @param t_initial_configuration default = {0,0,0}
    /// @returns the error
    const BLA::Matrix<3> JTranspIK(BLA::Matrix<3> x_des, BLA::Matrix<3, 3> t_gain, float t_dt, BLA::Matrix<3> t_initial_configuration = {0, 0, 0});

    /// @brief closed form IK
    /// @param pos desired position {x,y,z}
    /// @return
    BLA::Matrix<3> InverseKinematics(BLA::Matrix<3> pos);

    // NOTE - Unused Functions
    // BLA::Matrix<3> getDh(int a);
    // void resetInitialPos();

    // NOTE - undefined
    // void setBodyT(BLA::Matrix<4, 4> t_body_T);
    // BLA::Matrix<4, 4> body_T;

private:
    // Servos
    Servo shoulder;
    Servo knee;
    Servo ankle;

    // Zero Vectors and Polarity Vector
    float zeros[3]; // Where the angle of each servo corresponds to 0 in the DH params
    float polar[3]; // +/- 1 such that the servos rotate in the same dir as the DH Frames

    // Pins
    int pin_shoulder;
    int pin_knee;
    int pin_ankle;

    /// @brief computes Homogeneous transformation based on dh parameters
    BLA::Matrix<4, 4> dhTransform(float a, float alpha, float d, float theta);

    /// @brief computes fk and updates jacobians based on the current configuration
    void update_leg();

    void ForwardKinematics();

    void computeJacobian();

    // DH
    BLA::Matrix<3> dh_alpha;
    BLA::Matrix<3> dh_a;
    BLA::Matrix<3> dh_d;

    // Position vectors
    BLA::Matrix<3> p1;
    BLA::Matrix<3> p2;
    BLA::Matrix<3> pe; // Position of EndEffector in frame-0

    BLA::Matrix<4, 4> BaseFrameTranslation;

    // Translation between frames.
    BLA::Matrix<4, 4> T01;
    BLA::Matrix<4, 4> T12;
    BLA::Matrix<4, 4> T23;
    BLA::Matrix<4, 4> T34;

    BLA::Matrix<4, 4> T02;
    BLA::Matrix<4, 4> T03;
    BLA::Matrix<4, 4> T04;

    BLA::Matrix<3> z0;
    BLA::Matrix<3> z1;
    BLA::Matrix<3> z2;

    // Jacobians for Position, Orientation and total
    BLA::Matrix<3, 3> Jp;
    BLA::Matrix<3, 3> Jo;
    BLA::Matrix<6, 3> Jacobian;

    bool initialisation = false;

    /// @brief servo positions in radians {shoulder, knee, ankle}
    BLA::Matrix<3> theta;
};
