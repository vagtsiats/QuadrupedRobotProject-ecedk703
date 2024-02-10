#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <BasicLinearAlgebra.h>
#include "BLA_tools.h"

float evaluatePolynomial(const std::vector<float> &coefficients, float t);

std::vector<float> differentiatePolynomial(const std::vector<float> &coefficients);

std::vector<float> quintic_poly(const std::vector<float> &q1, const std::vector<float> &q2);
