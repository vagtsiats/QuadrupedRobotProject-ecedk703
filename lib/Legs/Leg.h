#include <ArduinoEigenDense.h>
using namespace Eigen;

class Leg
{
public:
    Leg(Matrix4d t_body_R);
    ~Leg();

private:
    const Matrix4d body_R;
    Vector3f dh_alpha;
    Vector3f dh_a;
    Vector3f dh_d;
    Vector3f pe;
    Matrix4f T01;
    Matrix4f T12;
    Matrix4f T23;
    MatrixXf Jacobian{6,3};

};
