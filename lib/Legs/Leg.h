#include <ArduinoEigenDense.h>
#include "Servo.h"
using namespace Eigen;

class Leg
{
public:
    Leg();
    ~Leg();

    void DriveLeg(int up, int mid, int low);

private:
    //Servos
    Servo shoulder;
    Servo knee;
    Servo ankle;
    //Position Relative to body
    const Matrix4d body_T;
    //DH params in vectors of 3
    Vector3f dh_alpha;
    Vector3f dh_a;
    Vector3f dh_d;
    //position of end effector in Frame-0 
    Vector3f pe;
    //Translation Matrices
    //To be computed from Forward Kin
    Matrix4f T01;
    Matrix4f T12;
    Matrix4f T23;
    //Jacobian matrice
    MatrixXf Jacobian{6,3};

};
