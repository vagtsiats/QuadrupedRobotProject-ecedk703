#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <BasicLinearAlgebra.h>
#include "BLA_tools.h"

float evaluatePolynomial(const std::vector<float> &coefficients, float t);

std::vector<float> differentiatePolynomial(const std::vector<float> &coefficients);

/// @brief Solving the following 3 equations for q1 and q2.   -> p(t) = q , p_dot(td) = qd , p_dot_dot(tdd) = qtt
/// @param q1 [q,t,  qd,td,  qdd,tdd]
/// @param q2 [q,t,  qd,td,  qdd,tdd]
/// @return coefficients of a quintic polynomial
std::vector<float> quintic_poly(const std::vector<float> &q1, const std::vector<float> &q2);
