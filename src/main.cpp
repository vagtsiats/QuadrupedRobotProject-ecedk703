#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"

using namespace BLA;
BLA::Matrix<4, 4> A;
Leg br(5,6,7);
void setup() {
    Serial.begin(9600);
    BLA::Matrix<3> br_dh_a={0.3,0.7,0.5};
    BLA::Matrix<3> br_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> br_dh_d={0,0,0};
    br.setDh(br_dh_a,br_dh_alpha,br_dh_d);
    Serial<<br.getDh(0);

}
void loop() {
}
