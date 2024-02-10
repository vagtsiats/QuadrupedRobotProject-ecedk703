#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include "polynomials.h"

class Turn
{
private:
    float vd;
    float L;
    float y0;
    float y1;

    float Tst;
    float Tsw;

    //For turn
    
    float n=6.6;
    float l=3.3;
    float x,y; 
    float phi=0.261799388;

    std::vector<float> P_x_coeffs;
    std::vector<float> P_y_coeffs;
    std::vector<float> Pd_x_coeffs;
    std::vector<float> Pd_y_coeffs;
    std::vector<float> P_z_coeffs;
    std::vector<float> Pd_z_coeffs;
    /// @brief calculates the trajectory's polynomials
    void calculate_trajectory();

public:
    Turn(float t_vd, float t_y0);
    float d;
    float theta1;
    float theta2;
    void change_parameters(float t_vd, float t_y0);
    //given phi in rad calculate d,theta1,theta2
    void calculateTurnParameters();
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
