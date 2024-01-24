#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "Quad.h"

using namespace BLA;
unsigned int time0;
Quad Robot;
void setup() {
    Serial.begin(9600);
    Robot.initHardware();
    time0=micros();
}

void loop() {
   Robot.inverseDiffKinematics(time0);
}
