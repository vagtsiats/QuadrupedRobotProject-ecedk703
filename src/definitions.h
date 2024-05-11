#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
// #include "Quad.h"
#include "Turn.h"
using namespace BLA;

const uint32_t LOOP_PERIODus = 100; // 0.0001sec
const float LOOP_PERIODsec = LOOP_PERIODus / 1e6;
uint32_t timer0; // constant loop timer
uint32_t timer1; // general purpose timer used for trajectory
uint32_t timer2; // init algorithm timer

// Quad Robot;
Leg testleg = Leg(11, 12, 13, {100, 90, 125}, {-1, -1, -1});

Matrix<3> init_error = {100, 100, 100};
float min_error = 1;
bool initialization = true;

Trajectory testtraj = Trajectory(4, -20);
