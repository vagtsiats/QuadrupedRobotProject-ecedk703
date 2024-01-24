#include "Quad.h"

Quad::Quad(/* args */)
:br(5,6,7,{90,90,90},{1,1,1}),
bl(2,3,4,{90,90,90},{1,1,1}),
fr(53,51,49,{90,90,90},{1,1,1}),
fl(47,45,43,{90,90,90},{1,1,1})
{
}

Quad::~Quad()
{}

void Quad::initHardware(){
    BLA::Matrix<3> br_dh_a={3,7,5};//in CM
    BLA::Matrix<3> br_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> br_dh_d={0,0,0};

    BLA::Matrix<3> bl_dh_a={3,7,5};//in CM
    BLA::Matrix<3> bl_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> bl_dh_d={0,0,0};

    BLA::Matrix<3> fr_dh_a={3,7,5};//in CM
    BLA::Matrix<3> fr_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> fr_dh_d={0,0,0};

    BLA::Matrix<3> fl_dh_a={3,7,5};//in CM
    BLA::Matrix<3> fl_dh_alpha={M_PI_2,0,0};
    BLA::Matrix<3> fl_dh_d={0,0,0};

    
    br.setDh(br_dh_a,br_dh_alpha,br_dh_d);
    br.attach_servos();

    bl.setDh(bl_dh_a,bl_dh_alpha,bl_dh_d);
    bl.attach_servos();

    fr.setDh(fr_dh_a,fr_dh_alpha,fr_dh_d);
    fr.attach_servos();

    fl.setDh(fl_dh_a,fl_dh_alpha,fl_dh_d);
    fl.attach_servos();

}
using namespace BLA;
Matrix<3> theta={0,-M_PI/4,-M_PI/4};
void Quad::inverseDiffKinematics(unsigned int time0){
    float t= micros()-time0;//in micros
    float t_sec=t/1e6;
    Matrix<3,3> K={1,0,0,0,1,0,0,0,1};
    float dt=0.02;

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