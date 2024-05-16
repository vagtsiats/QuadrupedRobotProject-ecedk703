#include "trajectory.h"

Trajectory::Trajectory(float t_vd, float t_y0, float t_Tsw, float t_L) : vd(t_vd), body_height(-t_y0), Tsw(t_Tsw), L(t_L)
{
    step_height = 2;
    calculate_trajectory();
}

// void Trajectory::change_parameters(float t_vd, float t_y0,float t_Tsw, float t_L)
// {
//     vd = t_vd;
//     body_height = t_y0;
//     calculate_trajectory();
// }

void Trajectory::calculate_trajectory()
{
    Tst = L / vd;
    float tsw_vx0 = Tsw / 4;

    P_x_coeffs = quintic_poly({-L / 2, 0, -vd, 0, 0, 0}, {0, Tsw / 2, 0, tsw_vx0, 0, Tsw / 2});
    Pd_x_coeffs = differentiatePolynomial(P_x_coeffs);

    P_y_coeffs = quintic_poly({body_height, 0, 0, 0, 0, 0}, {body_height + step_height, Tsw / 2, 0, Tsw / 2, 0, 3 * Tsw / 5});
    Pd_y_coeffs = differentiatePolynomial(P_y_coeffs);
}

BLA::Matrix<3> Trajectory::get_position(float t_t)
{
    t_t = fmod(t_t, get_T());

    if (t_t <= Tst)
    {
        return {((L / 2) - (vd * t_t)), 0, body_height};
    }
    else if ((t_t > Tst) && (t_t <= Tst + Tsw / 2))
    {
        return {evaluatePolynomial(P_x_coeffs, t_t - Tst), 0, evaluatePolynomial(P_y_coeffs, t_t - Tst)};
    }
    else if ((t_t > (Tst + Tsw / 2)) && (t_t <= Tst + Tsw))
    {
        return {-evaluatePolynomial(P_x_coeffs, Tsw + Tst - t_t), 0, evaluatePolynomial(P_y_coeffs, Tsw + Tst - t_t)};
    }

    return 0;
}

BLA::Matrix<3> Trajectory::get_velocity(float t_t)
{
    t_t = fmod(t_t, get_T());

    if (t_t <= Tst)
    {
        return {-vd, 0, 0};
    }
    else if ((t_t > Tst) && (t_t <= Tst + Tsw / 2))
    {
        return {evaluatePolynomial(Pd_x_coeffs, t_t - Tst), 0, evaluatePolynomial(Pd_y_coeffs, t_t - Tst)};
    }
    else if ((t_t > Tst + Tsw / 2) && (t_t <= Tst + Tsw))
    {
        return {evaluatePolynomial(Pd_x_coeffs, Tsw + Tst - t_t), 0, -evaluatePolynomial(Pd_y_coeffs, Tsw + Tst - t_t)};
    }

    return 0;
}
