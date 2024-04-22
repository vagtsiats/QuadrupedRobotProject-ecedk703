#include "Quad.h"

Quad::Quad(/* args */)
    : br(5, 6, 7, {90, 90, 90}, {1, 1, 1}),
      bl(2, 3, 4, {90, 90, 90}, {1, 1, 1}),
      fr(53, 51, 49, {90, 90, 90}, {1, 1, 1}),
      fl(47, 45, 43, {90, 90, 90}, {1, 1, 1}),
      traj(Trajectory(1, -14))
{
    walk_dt = {0, 2.0 / 4, 1.0 / 4, 3.0 / 4};
    walk_gain = BLAdiagonal<3>(5);
}

Quad::~Quad()
{
}

void Quad::initHardware()
{
    BLA::Matrix<3> br_dh_a = {3, 7, 5}; // in CM
    BLA::Matrix<3> br_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> br_dh_d = {0, 0, 0};

    BLA::Matrix<3> bl_dh_a = {3, 7, 5}; // in CM
    BLA::Matrix<3> bl_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> bl_dh_d = {0, 0, 0};

    BLA::Matrix<3> fr_dh_a = {3, 7, 5}; // in CM
    BLA::Matrix<3> fr_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> fr_dh_d = {0, 0, 0};

    BLA::Matrix<3> fl_dh_a = {3, 7, 5}; // in CM
    BLA::Matrix<3> fl_dh_alpha = {M_PI_2, 0, 0};
    BLA::Matrix<3> fl_dh_d = {0, 0, 0};

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

void Quad::gait(const double &t_time, float looptime)
{
    fl.JInvIK(traj.get_position(t_time + traj.get_T() * walk_dt[0]), traj.get_velocity(t_time + traj.get_T() * walk_dt[0]), walk_gain, looptime);
    fr.JInvIK(traj.get_position(t_time + traj.get_T() * walk_dt[1]), traj.get_velocity(t_time + traj.get_T() * walk_dt[1]), walk_gain, looptime);
    bl.JInvIK(traj.get_position(t_time + traj.get_T() * walk_dt[2]), traj.get_velocity(t_time + traj.get_T() * walk_dt[2]), walk_gain, looptime);
    br.JInvIK(traj.get_position(t_time + traj.get_T() * walk_dt[3]), traj.get_velocity(t_time + traj.get_T() * walk_dt[3]), walk_gain, looptime);
}