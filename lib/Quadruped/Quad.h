#include "Leg.h"
#include "BLA_tools.h"
#include "vector"
#include "trajectory.h"

class Quad
{
private:

    Leg bl;
    Leg fr;
    Leg fl;

    Trajectory traj;

    // NOTE - walking gait timing {FL, FR, BL, BR}
    std::vector<double> dt_walk;

public:
    void initHardware();
    Leg br;

    Quad(/* args */);
    ~Quad();

    void gait(double &t_time);
};
