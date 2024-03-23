#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
#include "Quad.h"
#include "Turn.h"
using namespace BLA;

const uint32_t LOOP_PERIODms = 1e4; // 0.01sec
const float LOOP_PERIODsec = LOOP_PERIODms / 1e6;
uint32_t timer0; // constant loop timer
double timer1;   // general purpose timer

// Quad Robot;

Leg testleg = Leg(47, 45, 43, {90, 90, 90}, {1, 1, 1});

Trajectory testtraj = Trajectory(1, -14);
