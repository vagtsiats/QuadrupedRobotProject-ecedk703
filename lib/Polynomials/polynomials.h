#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <BasicLinearAlgebra.h>
#include "BLA_tools.h"

double evaluatePolynomial(const std::vector<double> &coefficients, double t);

std::vector<double> differentiatePolynomial(const std::vector<double> &coefficients);

std::vector<double> quintic_poly(const std::vector<double> &q1, const std::vector<double> &q2);
