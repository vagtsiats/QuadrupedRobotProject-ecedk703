#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
using namespace BLA;
unsigned int time0;
Leg br(5,6,7);

void setup() {
    Serial.begin(9600);
    BLA::Matrix<3> br_dh_a={3,7,5};//in CM
    BLA::Matrix<3> br_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> br_dh_d={0,0,0};

    br.setDh(br_dh_a,br_dh_alpha,br_dh_d);

    time0=micros();
}
Matrix<3> theta={0,-M_PI/4,-M_PI/4};
Matrix<3,3> K={1,0,0,0,1,0,0,0,1};
float dt=0.02;
float t;

void loop() {
    t= micros()-time0;//in micros
    float t_sec=t/1e6;
    br.updateTranslations(theta);
    br.computeJacobian();

    //For theta 0 -pi/4 -pi/4 -> 7.95 0 -9.95 so we want to move on the z axis only with inverse dif kinematics: 
    Matrix<3> xd_dot={0,0,1};
    Matrix<3> xd={7.95,0,-9.95+1*t_sec};  
    Matrix<3> error=xd-br.forwardKinematics();
    theta+=(Inverse(br.getJacobianPos())*(xd_dot+K*error))*dt;
    printMatrix(br.forwardKinematics());
    delay(20);
}
