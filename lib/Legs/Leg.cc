#include "Leg.h"
using namespace BLA;

Leg::Leg(int pin_shoulder , int pin_knee,int pin_ankle)
{
    shoulder.attach(pin_shoulder);
    knee.attach(pin_knee);
    ankle.attach(pin_ankle);

}

Leg::~Leg()
{
}
void Leg::setDh(BLA::Matrix<3> t_dh_a,BLA::Matrix<3> t_dh_alpha,BLA::Matrix<3> t_dh_d)
{
    dh_a=t_dh_a;
    dh_alpha=t_dh_alpha;  
    dh_d=t_dh_d;
}
void Leg::DriveLeg(int up,int mid ,int low){

    shoulder.write(up);
    knee.write(mid);
    ankle.write(low);

}
BLA::Matrix<3> Leg::getDh(int a){
    if(a==0){return dh_a;}
    else if(a==1){return dh_alpha;}
    else{return dh_d;}
}
Matrix<4,4> Leg::dhTransform(float a, float alpha, float d, float theta) {
    Matrix<4,4> T = {cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha), sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha), 0, 0, 0, 1};
    return T;
}

Matrix<4,4> Leg::forwardKinematics( Matrix<3> theta) {
    Matrix<4,4> final_transform = dhTransform(dh_a(0), dh_alpha(0), dh_d(0), theta(0));

    for (int i = 1; i < 3; i++) {
      final_transform = final_transform * dhTransform(dh_a(i), dh_alpha(i), dh_d(i), theta(i));
    }

    return final_transform;
}