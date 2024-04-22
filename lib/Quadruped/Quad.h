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

    // NOTE - walking gait timing {FL, FR, BL, BR}
    std::vector<double> walk_dt;

    BLA::Matrix<3, 3> walk_gain;

public:
    void
    initHardware();

    Quad(/* args */);
    ~Quad();

    void gait(const double &t_time, float looptime);
};
