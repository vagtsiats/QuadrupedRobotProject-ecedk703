#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"

using namespace BLA;
BLA::Matrix<4, 4> A;
Leg br(5,6,7);

void printMatrix(Matrix<4, 4> matrix);
void setup() {
    Serial.begin(9600);
    BLA::Matrix<3> br_dh_a={3,7,5};//in CM
    BLA::Matrix<3> br_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> br_dh_d={0,0,0};
    br.setDh(br_dh_a,br_dh_alpha,br_dh_d);
    br.updateTranslations(br_dh_d);
    br.computeJacobian();
}
void loop() {
    
}
void printMatrix(Matrix<4, 4> matrix) {
  for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(matrix(i, j));
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
    Serial.println();
}