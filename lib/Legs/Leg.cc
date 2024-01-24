#include "Leg.h"
using namespace BLA;
Leg::Leg(int t_pin_shoulder , int t_pin_knee,int t_pin_ankle, const std::vector<float>& t_zeros, const std::vector<float>& t_polar)
{
    pin_shoulder=t_pin_shoulder;
    pin_knee=t_pin_knee;
    pin_ankle=t_pin_ankle;
    for(int i=0;i<3;i++){
        zeros[i]=t_zeros[i];
        polar[i]=t_polar[i]; 
    }
    
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

    shoulder.write(polar[0]*(up-zeros[0]));
    knee.write(polar[1]*(mid-zeros[1]));
    ankle.write(polar[2]*(low-zeros[2]));

}
BLA::Matrix<3> Leg::getDh(int a){
    if(a==0){return dh_a;}
    else if(a==1){return dh_alpha;}
    else{return dh_d;}
}
Matrix<4,4> Leg::dhTransform(float a, float alpha, float d, float theta) {
    Matrix<4,4> T = {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
                    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta), 
                    0, sin(alpha), cos(alpha), d , 
                    0, 0, 0, 1};
    return T;
}
void Leg::updateTranslations( BLA::Matrix<3> theta) {
    T01= dhTransform(dh_a(0), dh_alpha(0), dh_d(0), theta(0));
    T12 = dhTransform(dh_a(1), dh_alpha(1), dh_d(1), theta(1));
    T23= dhTransform(dh_a(2), dh_alpha(2), dh_d(2), theta(2));
    T02=T01*T12;
    T03=T02*T23;

    pe(0)=T03(0,3);
    pe(1)=T03(1,3);
    pe(2)=T03(2,3);    
    return;
}
Matrix<3> Leg::forwardKinematics( ) {
    return pe;
}
void Leg::computeJacobian(){
    z1(0)=T01(0,2);
    z1(1)=T01(1,2);
    z1(2)=T01(2,2);

    z2(0)=T02(0,2);
    z2(1)=T02(1,2);
    z2(2)=T02(2,2);

    p1(0)=T01(0,3);
    p1(1)=T01(1,3);
    p1(2)=T01(2,3);

    p2(0)=T02(0,3);
    p2(1)=T02(1,3);
    p2(2)=T02(2,3);

    Jp=crossProduct(z0,pe)||crossProduct(z1,pe-p1)||crossProduct(z2,pe-p2);
 
    Jo=z0||z1||z2;

    Jacobian=Jp && Jo;
    return;

}
Matrix<6, 3> Leg::getJacobian(){

    return Jacobian;
}

Matrix<3, 3> Leg::getJacobianPos(){

    return Jp;
}


Matrix<3> Leg::crossProduct(Matrix<3> a , Matrix<3> b ){
    Matrix<3> cross;
    cross(0)=a(1)*b(2)-a(2)*b(1);
    cross(1)=a(0)*b(2)-a(2)*b(0);
    cross(2)=a(0)*b(1)-a(1)*b(0);

    return cross;
}
void Leg::inverseDiffKinematics(Matrix<3> theta0,Matrix<3> xd,  Matrix<3> xd_dot){
    
    Matrix<3,3> K={1,0,0,
                   0,1,0,
                   0,0,1};
    float dt=0.02;
    
    if(initialisation){  
        theta=theta0;
        initialisation=false;
    }

    updateTranslations(theta);
    computeJacobian();

    //For theta 0 -pi/4 -pi/4 -> 7.95 0 -9.95 so we want to move on the z axis only with inverse dif kinematics: 
    Matrix<3> error=xd-forwardKinematics();
    theta+=(Inverse(getJacobianPos())*(xd_dot+K*error))*dt;
    printMatrix(forwardKinematics());
    
}

void Leg::resetInitialPos(){
    initialisation=false;
    return;
}