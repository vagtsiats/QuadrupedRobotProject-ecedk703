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
