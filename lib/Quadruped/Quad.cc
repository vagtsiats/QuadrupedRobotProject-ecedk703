#include "Quad.h"

Quad::Quad(/* args */)
    : br(5, 6, 7, {90, 90, 90}, {1, 1, 1}),
      bl(2, 3, 4, {90, 90, 90}, {1, 1, 1}),
      fr(53, 51, 49, {90, 90, 90}, {1, 1, 1}),
      fl(47, 45, 43, {90, 90, 90}, {1, 1, 1}),
      body_height(20),
      Tsw(0.5),
      traj(Trajectory(1, body_height, Tsw, 3 * Tsw * 1))
{
    // NOTE - gait timing {FL, FR, BL, BR}
    walk_dt = {0, 2. / 4, 1. / 4, 3. / 4};
    walk_dt = {0, 1. / 2, 1. / 2, 0};

    gait_gain = BLAdiagonal<3>(5);
}

// void Quad::set_velocity(float t_vd)
// {
//     // traj = Trajectory(t_vd, -body_height, );
// }

void Quad::initHardware()
{
    BLA::Matrix<3> br_dh_a = {0, 11, 13}; // in CM
    BLA::Matrix<3> br_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> br_dh_d = {0, 5, 0};

    BLA::Matrix<3> bl_dh_a = {0, 11, 13}; // in CM
    BLA::Matrix<3> bl_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> bl_dh_d = {0, -5, 0};

    BLA::Matrix<3> fr_dh_a = {0, 11, 13}; // in CM
    BLA::Matrix<3> fr_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> fr_dh_d = {0, 5, 0};

    BLA::Matrix<3> fl_dh_a = {0, 11, 13}; // in CM
    BLA::Matrix<3> fl_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> fl_dh_d = {0, -5, 0};

    br.setDh(br_dh_a, br_dh_alpha, br_dh_d);
    br.attach_servos();

    bl.setDh(bl_dh_a, bl_dh_alpha, bl_dh_d);
    bl.attach_servos();

    fr.setDh(fr_dh_a, fr_dh_alpha, fr_dh_d);
    fr.attach_servos();

    fl.setDh(fl_dh_a, fl_dh_alpha, fl_dh_d);
    fl.attach_servos();

    BLA::Matrix<3> init_th = {0, 0, 0};
    br.DriveLeg(init_th);
    bl.DriveLeg(init_th);
    fr.DriveLeg(init_th);
    fl.DriveLeg(init_th);
}

void Quad::init_walk(float vd)
{
    traj = Trajectory(vd, body_height, Tsw, 3 * Tsw * vd);
    dt = trot_dt;

    // TODO - Add inverse kinematics
}

void Quad::init_walk(float vd)
{
    traj = Trajectory(vd, body_height, Tsw, Tsw * vd);
    dt = walk_dt;

    // TODO - Add inverse kinematics
}

void Quad::walk(const double &t_time, float t_looptime)
{
    fr.JInvIK(traj.get_position(t_time + traj.get_T() * dt[1]), traj.get_velocity(t_time + traj.get_T() * dt[1]), gait_gain, t_looptime);
    bl.JInvIK(traj.get_position(t_time + traj.get_T() * dt[2]), traj.get_velocity(t_time + traj.get_T() * dt[2]), gait_gain, t_looptime);
    fl.JInvIK(traj.get_position(t_time + traj.get_T() * dt[0]), traj.get_velocity(t_time + traj.get_T() * dt[0]), gait_gain, t_looptime);
    br.JInvIK(traj.get_position(t_time + traj.get_T() * dt[3]), traj.get_velocity(t_time + traj.get_T() * dt[3]), gait_gain, t_looptime);
}
