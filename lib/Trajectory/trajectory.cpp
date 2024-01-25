#include "trajectory.h"

Trajectory::Trajectory(double t_vd, double t_L, double t_y0) : vd(t_vd), L(t_L), y0(t_y0)
{
    y1 = 2;
    Tsw = 3;

    calculate_trajectory();
}

void Trajectory::change_parameters(double t_vd, double t_L, double t_y0)
{
    vd = t_vd;
    L = t_L;
    y0 = t_y0;
    calculate_trajectory();
}

void Trajectory::calculate_trajectory()
{
    Tst = L / vd;

    P_x_coeffs = quintic_poly({-L / 2, 0, -vd, 0, 0, 0}, {0, Tsw / 2, 0, Tsw / 4, 0, Tsw / 2});
    Pd_x_coeffs = differentiatePolynomial(P_x_coeffs);

    // printvector(P_x_coeffs);
    // Serial.println();

    // printvector(Pd_x_coeffs);
    // Serial.println();

    P_y_coeffs = quintic_poly({y0, 0, 0, 0, 0, 0}, {y0 + y1, Tsw / 2, 0, Tsw / 2, 0, 3 * Tsw / 5});
    Pd_y_coeffs = differentiatePolynomial(P_y_coeffs);
}

BLA::Matrix<1, 3, double> Trajectory::get_position(double t_t)
{
    if (t_t <= Tst)
    {
        return {((L / 2) - (vd * t_t)), 0, y0};
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

BLA::Matrix<1, 3, double> Trajectory::get_velocity(double t_t)
{
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
