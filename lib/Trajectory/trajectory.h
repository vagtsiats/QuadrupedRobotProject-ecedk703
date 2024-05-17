#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include "polynomials.h"

class Trajectory
{
private:
    float vd;
    float L; // walk_L = 3*Tsw*vd \ trot_L = Tsw*vd
    float body_height;
    float step_height;
    float D;

    float Tst;
    float Tsw;

    std::vector<float> P_x_coeffs;
    std::vector<float> P_y_coeffs;
    std::vector<float> Pd_x_coeffs;
    std::vector<float> Pd_y_coeffs;

    /// @brief calculates the trajectory's polynomials
    void calculate_trajectory();

public:
    /// @brief Initializes a trajectory for the legs
    /// @param t_vd desired velocity
    /// @param t_y0 body distance from the ground -> has to be positive values
    /// @param t_Tsw swing time in seconds
    /// @param t_L step length
    Trajectory(float t_vd, float t_y0, float t_Tsw, float t_L);

    float get_T() { return Tst + Tsw; }

    /// @brief calculates the desired position @ t
    /// @param t elapsed time,    t=0 is the touch-down event
    /// @return Returns 1x3 Matrix with {x,y,z}, x+ the direction of movement
    BLA::Matrix<3> get_position(float t_t);

    /// @brief calculates the desired velocity @ t
    /// @param t elapsed time,    t=0 is the touch-down event
    /// @return
    BLA::Matrix<3> get_velocity(float t_t);
};
