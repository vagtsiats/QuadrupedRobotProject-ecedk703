#include "Quad.h"

Quad::Quad(/* args */)
    : br(16, 15, 14, {70, 92, 70}, {-1, 1, 1}, 0),
      bl(17, 18, 19, {95, 95, 130}, {-1, -1, -1}, 1),
      fr(11, 12, 13, {80, 92, 70}, {-1, 1, 1}, 0),
      fl(10, 9, 8, {100, 90, 125}, {-1, -1, -1}, 1),
      body_height(20),
      Tsw(0.25),
      u(0),
      traj(Trajectory(1, body_height, Tsw, 3 * Tsw * 1))
{
    // NOTE - gait timing {FL, FR, BL, BR}
    walk_dt = {0, 2. / 4, 1. / 4, 3. / 4};
    trot_dt = {0, 1. / 2, 1. / 2, 0};

    offset_fl = {0, 5, 0};
    offset_fr = {0, -5, 0};
    offset_bl = {-2, 5, 0};
    offset_br = {-2, -5, 0};

    gait_gain = BLAdiagonal<3>(1e5);
}

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

    // BLA::Matrix<3> init_th = {0, 0, 0};
    // br.driveLeg(init_th);
    // bl.driveLeg(init_th);
    // fr.driveLeg(init_th);
    // fl.driveLeg(init_th);
}

void Quad::init_trot(float vd)
{
    traj = Trajectory(vd, body_height, Tsw, 3 * Tsw * vd);
    traj_T = traj.get_T();
    dt = trot_dt;

    drive_legs_IK(traj.get_position(traj_T * (u + dt[0])) + offset_fl,
                  traj.get_position(traj_T * (u + dt[1])) + offset_fr,
                  traj.get_position(traj_T * (u + dt[2])) + offset_bl,
                  traj.get_position(traj_T * (u + dt[3])) + offset_br);
}

void Quad::init_walk(float vd)
{
    traj = Trajectory(vd, body_height, Tsw, Tsw * vd);
    traj_T = traj.get_T();
    dt = walk_dt;

    drive_legs_IK(traj.get_position(traj_T * (u + dt[0])) + offset_fl,
                  traj.get_position(traj_T * (u + dt[1])) + offset_fr,
                  traj.get_position(traj_T * (u + dt[2])) + offset_bl,
                  traj.get_position(traj_T * (u + dt[3])) + offset_br);
}

void Quad::walk(const double &t_time, float t_looptime)
{

    set_time(t_time);

    fl.JInvIK(traj.get_position(traj_T * (u + dt[0])) + offset_fl, traj.get_velocity(traj_T * (u + dt[0])), gait_gain, t_looptime);
    fr.JInvIK(traj.get_position(traj_T * (u + dt[1])) + offset_fr, traj.get_velocity(traj_T * (u + dt[1])), gait_gain, t_looptime);
    bl.JInvIK(traj.get_position(traj_T * (u + dt[2])) + offset_bl, traj.get_velocity(traj_T * (u + dt[2])), gait_gain, t_looptime);
    br.JInvIK(traj.get_position(traj_T * (u + dt[3])) + offset_br, traj.get_velocity(traj_T * (u + dt[3])), gait_gain, t_looptime);

    drive_legs();
}

void Quad::drive_legs()
{
    fl.driveLeg();
    fr.driveLeg();
    bl.driveLeg();
    br.driveLeg();
}

void Quad::drive_legs(BLA::Matrix<3> q_fl, BLA::Matrix<3> q_fr, BLA::Matrix<3> q_bl, BLA::Matrix<3> q_br)
{
    fl.driveLeg(q_fl);
    fr.driveLeg(q_fr);
    bl.driveLeg(q_bl);
    br.driveLeg(q_br);
}

void Quad::drive_legs_IK(BLA::Matrix<3> p_fl, BLA::Matrix<3> p_fr, BLA::Matrix<3> p_bl, BLA::Matrix<3> p_br)
{

    fl.InverseKinematics(p_fl);
    fr.InverseKinematics(p_fr);
    bl.InverseKinematics(p_bl);
    br.InverseKinematics(p_br);

    drive_legs();
}

void Quad::set_time(const float &t_time)
{
    u = fmod(t_time, traj_T) / traj_T;
}
