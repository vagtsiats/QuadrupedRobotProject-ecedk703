#pragma once;

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
#include "Quad.h"
using namespace BLA;

unsigned int time0;
Quad Robot;

Matrix<3> theta0 = {0, -M_PI / 4, -M_PI / 4};
float t;

Matrix<3> xd(float t){
    Matrix<3> xd={7.95,0,-9.95+1*t};  
    return xd;
}
Matrix<3> xd_dot(float t){
    Matrix<3> xd_dot={0,0,1};
    return xd_dot;
}
