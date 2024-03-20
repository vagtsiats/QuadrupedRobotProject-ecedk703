#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include <iostream>

void printvector(const std::vector<float> &a);

// Returns Angle in Rad with sides a,b,c where c is opposite to the angle
float CosineTheoremAngle(float a, float b, float c);

// Returns Side length in cm with sides a,b, and angle phi in rad
float CosineTheoremSide(float a, float b, float phi);
float QuadEqSolution(float a, float b, float c);
float rad2deg(float rad);
float deg2rad(float degrees);
BLA::Matrix<3> crossProduct(BLA::Matrix<3> a, BLA::Matrix<3> b);

template <int MatDim, typename type = float>
BLA::Matrix<MatDim, MatDim, type> BLAdiagonal(float k)
{
    BLA::Matrix<MatDim, MatDim> eye;

    eye.Fill(0);

    for (int i = 0; i < MatDim; i++)
    {
        for (int j = 0; j < MatDim; j++)
        {
            if (i == j)
                eye(i, j) = k;
        }
    }

    return eye;
}

template <int MatCols, typename type>
std::vector<float> BLAMatrix2stdVector(const BLA::Matrix<1, MatCols, type> &Mat)
{

    std::vector<float> ret(MatCols);

    for (int i = 0; i < MatCols; i++)
    {
        ret[i] = Mat(0, i);
    }
    return ret;
}

template <int MatRows, int MatCols>
BLA::Matrix<MatCols, MatRows> BLApseudoInverse(const BLA::Matrix<MatRows, MatCols> &Mat)
{
    return Inverse(~Mat * Mat) * (~Mat);
}

template <int MatRows, int MatCols, typename type>
void BLAprintMatrix(const BLA::Matrix<MatRows, MatCols, type> &matrix)
{
    Serial.print("[");
    for (int i = 0; i < MatRows; i++)
    {
        for (int j = 0; j < MatCols; j++)
        {
            Serial.print(matrix(i, j));
            if (j != MatCols - 1)
                Serial.print("\t");
        }
        if (i == MatRows - 1)
            Serial.print("]");
        else
            Serial.println();
    }
    // Serial.println();
}
