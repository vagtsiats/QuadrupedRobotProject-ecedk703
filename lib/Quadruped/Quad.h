#include "Leg.h"
#include "BLA_tools.h"
#include "vector"
#include "trajectory.h"

class Quad
{
private:
    Trajectory traj;

    float body_height;
    float Tsw;
    float traj_T;

    // NOTE - gait timing {FL, FR, BL, BR}
    std::vector<double> walk_dt;
    std::vector<double> trot_dt;
    std::vector<double> dt;
    float u; // trajectory period percentage [0,1]
    void set_time(const float &t_time);

    BLA::Matrix<3, 3> gait_gain;

    BLA::Matrix<3> offset_fl;
    BLA::Matrix<3> offset_fr;
    BLA::Matrix<3> offset_bl;
    BLA::Matrix<3> offset_br;

public:
    Leg br;
    Leg bl;
    Leg fr;
    Leg fl;

    void initHardware();

    Quad(/* args */);

    void init_walk(float vd);
    void init_trot(float vd);
    void drive_legs();

    /// @brief Drive all legs to desired theta
    /// @param q_fl
    /// @param q_fr
    /// @param q_bl
    /// @param q_br
    void drive_legs(BLA::Matrix<3> q_fl, BLA::Matrix<3> q_fr, BLA::Matrix<3> q_bl, BLA::Matrix<3> q_br);

    /// @brief Drive all legs to desired position using IK
    void drive_legs_IK(BLA::Matrix<3> p_fl, BLA::Matrix<3> p_fr, BLA::Matrix<3> p_bl, BLA::Matrix<3> p_br);

    void gait(const double &t_time, float t_looptime);
};
