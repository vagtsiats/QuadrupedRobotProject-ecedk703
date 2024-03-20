#pragma once

#include <BasicLinearAlgebra.h>

// void printvector(const std::vector<float> &a);
float CosineTheoremAngle(float a,float b, float c);
float CosineTheoremSide(float a,float b, float phi);
float QuadEqSolution(float a ,float b, float c);
float rad2deg(float rad);
float deg2rad(float degrees);


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
