#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"

class Quad
{
private:

    

    Trajectory traj;

    // NOTE - walking gait timing {FL, FR, BL, BR}
 

public:
    void initHardware();
    Leg br;
    Leg bl;
    Leg fr;
    Leg fl;
    Quad(/* args */);
    ~Quad();

    void gait(double &t_time);
};
