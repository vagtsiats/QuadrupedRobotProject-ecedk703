#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include "polynomials.h"

class Trajectory
{
private:
    float vd;
    float L;
    float y0;
    float y1;

    float Tst;
    float Tsw;

    std::vector<float> P_x_coeffs;
    std::vector<float> P_y_coeffs;
    std::vector<float> Pd_x_coeffs;
    std::vector<float> Pd_y_coeffs;

    /// @brief calculates the trajectory's polynomials
    void calculate_trajectory();

public:
    Trajectory(float t_vd, float t_y0);

    void change_parameters(float t_vd, float t_y0);

    float get_T() { return Tst + Tsw; }

    /// @brief calculates the desired position @ t
    /// @param t > 0<t<Tst+Tsw,    t=0 is the touch-down event
    /// @return Returns 1x3 Matrix with {x,y,z}, x+ the direction of movement
    BLA::Matrix<3> get_position(float t_t);

    /// @brief calculates the desired velocity @ t
    /// @param t > 0<t<Tst+Tsw
    /// @return
    BLA::Matrix<3> get_velocity(float t_t);
};
