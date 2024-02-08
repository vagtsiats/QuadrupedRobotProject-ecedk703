#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
#include "Quad.h"
#include "Turn.h"
using namespace BLA;

const u_int32_t LOOP_PERIODms = 1e4; // 0.01sec
const float LOOP_PERIODsec = LOOP_PERIODms / 1e6;
u_int32_t timer0; // constant loop timer
double timer1;    // general purpose timer

Quad Robot;

// Matrix<3> theta0 = {0, M_PI/6,-M_PI/4};
// float t;

// NOTE - DiffKinematics Test
//  Matrix<3> theta0 = {0, -M_PI / 4, -M_PI / 4};

// Matrix<3> xd(float t)
// {
//     Matrix<3> xd = {7.95, 0, -9.95 + 1 * t};
//     return xd;
// }
// Matrix<3> xd_dot(float t)
// {
//     Matrix<3> xd_dot = {0, 0, 1};
//     return xd_dot;
// }
