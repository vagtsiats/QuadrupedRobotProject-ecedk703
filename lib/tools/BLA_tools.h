#pragma once

#include <BasicLinearAlgebra.h>
#include <vector>

template <int MatCols>
std::vector<double> BLAMatrix2stdVector(const BLA::Matrix<1, MatCols> &Mat)
{

    std::vector<double> ret(MatCols);

    for (int i = 0; i < MatCols; i++)
    {
        ret[i] = Mat(i);
    }
    return ret;
};

template <int MatRows, int MatCols>
BLA::Matrix<MatCols, MatRows> BLApseudoInverse(const BLA::Matrix<MatRows, MatCols> &Mat)
{
    return Inverse(~Mat * Mat) * (~Mat);
}

template <int MatRows, int MatCols>
void printMatrix(const BLA::Matrix<MatRows, MatCols> &matrix)
{
    for (int i = 0; i < MatRows; i++)
    {
        for (int j = 0; j < MatCols; j++)
        {
            Serial.print(matrix(i, j));
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
    Serial.println();
}
