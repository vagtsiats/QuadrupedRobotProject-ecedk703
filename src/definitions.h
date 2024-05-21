#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
#include "Quad.h"
#include <SPI.h>
#include <WiFiNINA.h>

using namespace BLA;

const uint32_t LOOP_PERIODus = 200; // 0.0001sec
const float LOOP_PERIODsec = LOOP_PERIODus / 1e6;
uint32_t timer_0; // constant loop timer
uint32_t timer1;  // general purpose timer used for trajectory
uint32_t timer2;  // init algorithm timer

Matrix<3> conf = {0, 0., 0.};
Matrix<3> pos = {0, 0, -20};

Quad Robot;

const char *ssid = "Xiaomi 12";
const char *password = "paparia2018";

WiFiServer server(23);
WiFiClient client;

float SPEED = 2.0;
bool Stand = true;
