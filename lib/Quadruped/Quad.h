#include "Leg.h"
#include "BLA_tools.h"
#include "vector"
#include "trajectory.h"

class Quad
{
private:
    Leg br;
    Leg bl;
    Leg fr;
    Leg fl;

    Trajectory traj;

    float body_height;
    float Tsw;

    // NOTE - gait timing {FL, FR, BL, BR}
    std::vector<double> walk_dt;
    std::vector<double> trot_dt;
    std::vector<double> dt;

    BLA::Matrix<3, 3> gait_gain;

public:
    void
    initHardware();

    Quad(/* args */);
    ~Quad();

    void set_velocity(float t_vd);

    void init_walk(float vd);
    void init_trot(float vd);

    void walk(const double &t_time, float t_looptime);
    void trot(const double &t_time, float t_looptime);
};
