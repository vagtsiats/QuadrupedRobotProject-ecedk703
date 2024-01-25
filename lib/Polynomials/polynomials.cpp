#include "polynomials.h"

double evaluatePolynomial(const std::vector<double> &coefficients, double t)
{
    double result = 0.0;
    int degree = coefficients.size() - 1;

    for (int i = degree; i >= 0; --i)
    {
        result += coefficients[i] * pow(t, degree - i);
    }

    return result;
}

std::vector<double> differentiatePolynomial(const std::vector<double> &coefficients)
{
    int degree = coefficients.size() - 1;
    std::vector<double> derivativeCoefficients(degree);

    for (int i = degree; i > 0; --i)
    {
        derivativeCoefficients[degree - i] = i * coefficients[i];
    }

    return derivativeCoefficients;
}

// Returns the coefficients of a quintic polynomial by solving the following 6 equations
//  p(t) = q , p_dot(td) = qd , p_dot_dot(tdd) = qtt
//  q1 = [q,t,  qd,td,  qdd,tdd]
//  q2 = [q,t,  qd,td,  qdd,tdd]
std::vector<double> quintic_poly(const std::vector<double> &q1, const std::vector<double> &q2)
{

    BLA::Matrix<6, 6, double> T = {pow(q1[1], 5), pow(q1[1], 4), pow(q1[1], 3), pow(q1[1], 2), q1[1], 1,
                                   pow(q2[1], 5), pow(q2[1], 4), pow(q2[1], 3), pow(q2[1], 2), q2[1], 1,
                                   5 * pow(q1[3], 4), 4 * pow(q1[3], 3), 3 * pow(q1[3], 2), 2 * q1[3], 1, 0,
                                   5 * pow(q2[3], 4), 4 * pow(q2[3], 3), 3 * pow(q2[3], 2), 2 * q2[3], 1, 0,
                                   20 * pow(q1[5], 3), 12 * pow(q1[5], 2), 6 * q1[5], 2, 0, 0,
                                   20 * pow(q2[5], 3), 12 * pow(q2[5], 2), 6 * q2[5], 2, 0, 0};

    BLA::Matrix<6, 1, double> g = {q1[0],
                                   q2[0],
                                   q1[2],
                                   q2[2],
                                   q1[4],
                                   q2[4]};

    BLA::Matrix<1, 6, double> a = ~(BLA::Inverse(T) * g);

    auto retmat = BLAMatrix2stdVector<6>(a);

    return retmat;
}
