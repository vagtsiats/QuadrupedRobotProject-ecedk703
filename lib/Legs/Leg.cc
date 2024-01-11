#include "Leg.h"
using namespace BLA;
Leg::Leg(int t_pin_shoulder , int t_pin_knee,int t_pin_ankle)
{
    pin_shoulder=t_pin_shoulder;
    pin_knee=t_pin_knee;
    pin_ankle=t_pin_ankle;

}

Leg::~Leg()
{
}
void Leg::attach_servos(){
    shoulder.attach(pin_shoulder);
    knee.attach(pin_knee);
    ankle.attach(pin_ankle);
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
void Leg::updateTranslations( BLA::Matrix<3> theta) {
    T01= dhTransform(dh_a(0), dh_alpha(0), dh_d(0), theta(0));
    T02 = T01 * dhTransform(dh_a(1), dh_alpha(1), dh_d(1), theta(1));
    T03= T02*dhTransform(dh_a(2), dh_alpha(2), dh_d(2), theta(2));
    return;
}
Matrix<4,4> Leg::forwardKinematics( ) {
    return T03;
}
Matrix<6,3> Leg::computeJacobian(){
    pe(0)=T03(0,3);
    pe(1)=T03(1,3);
    pe(2)=T03(2,3);
    
    z1(0)=T01(0,2);
    z1(1)=T01(1,2);
    z1(2)=T01(2,2);

    z2(0)=T02(0,2);
    z2(1)=T02(1,2);
    z2(2)=T02(2,2);


    return;

}

Matrix<3> Leg::crossProduct(Matrix<3> a , Matrix<3> b ){
    Matrix<3> cross;
    cross(0)=a(1)*b(2)-a(2)*b(1);
    cross(1)=a(0)*b(2)-a(2)*b(0);
    cross(2)=a(0)*b(1)-a(1)*b(0);

    return cross;
}