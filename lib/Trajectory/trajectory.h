#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include "polynomials.h"

class Trajectory
{
private:
    double vd;
    double L;
    double y0;
    double y1;

    double Tst;
    double Tsw;

    std::vector<double> P_x_coeffs;
    std::vector<double> P_y_coeffs;
    std::vector<double> Pd_x_coeffs;
    std::vector<double> Pd_y_coeffs;

    /// @brief calculates the trajectory's polynomials
    void calculate_trajectory();

public:
    Trajectory(double t_vd, double t_L, double t_y0);

    void change_parameters(double t_vd, double t_L, double t_y0);

    double get_T() { return Tst + Tsw; }

    /// @brief calculates the desired position @ t
    /// @param t > 0<t<Tst+Tsw
    /// @return
    BLA::Matrix<1, 3> get_position(double t_t);

    /// @brief calculates the desired velocity @ t
    /// @param t > 0<t<Tst+Tsw
    /// @return
    BLA::Matrix<1, 3, double> get_velocity(double t_t);
};
