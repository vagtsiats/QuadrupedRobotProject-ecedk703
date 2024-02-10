#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>
#include <iostream>

void printvector(const std::vector<float> &a);

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
