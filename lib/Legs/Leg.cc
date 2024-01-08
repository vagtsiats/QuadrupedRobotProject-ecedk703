#include "Leg.h"
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