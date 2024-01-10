#pragma once

#include "Servo.h"
#include <BasicLinearAlgebra.h>

class Leg
{
public:
    Leg(int pin_shoulder,int pin_knee,int pin_ankle);
    ~Leg();

    void setDh( BLA::Matrix<3> t_dh_a,BLA::Matrix<3> t_dh_alpha, BLA::Matrix<3> t_dh_d);

    void setBodyT(BLA::Matrix<4, 4> t_body_T);

    void DriveLeg(int up, int mid, int low);
    
    BLA::Matrix<3> getDh(int a);

    BLA::Matrix<4, 4> dhTransform(float a, float alpha, float d, float theta);

    void updateTranslations(BLA::Matrix<3> theta);

    BLA::Matrix<4, 4> forwardKinematics();

    BLA::Matrix<6, 3> computeJacobian();

    BLA::Matrix<3> crossProduct(BLA::Matrix<3> a, BLA::Matrix<3> b);

private:
    //Servos
    Servo shoulder;
    Servo knee;
    Servo ankle;
    //Position Relative to body
    BLA::Matrix<4,4> body_T;
    //DH
    BLA::Matrix<3> dh_alpha;
    BLA::Matrix<3> dh_a;
    BLA::Matrix<3> dh_d;
    //Position of End effector in frame-0
    BLA::Matrix<3> pe;
    //Translation between frames.
    BLA::Matrix<4, 4> T01;
    BLA::Matrix<4, 4> T12;
    BLA::Matrix<4, 4> T23;
    BLA::Matrix<4, 4> T02;
    BLA::Matrix<4, 4> T03;

    BLA::Matrix<3> z0={0,0,1};
    Matrix<3> z1;
    Matrix<3> z2;

    BLA::Matrix<6, 3> Jacobian;


};

