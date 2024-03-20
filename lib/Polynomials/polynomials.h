#pragma once


#include <BasicLinearAlgebra.h>
#include "BLA_tools.h"

float evaluatePolynomial(BLA::Matrix<6> coefficients, float t);

BLA::Matrix<6> differentiatePolynomial(BLA::Matrix<6> coefficients);

BLA::Matrix<6> quintic_poly(BLA::Matrix<6> q1, BLA::Matrix<6>q2);
