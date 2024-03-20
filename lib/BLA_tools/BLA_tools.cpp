#include <BLA_tools.h>

void printvector(const std::vector<double> &a)
{
    Serial.print("[");
    for (int i = 0; i < a.size(); i++)
    {
        if (i != 0)
            Serial.print(",");
        Serial.print(a[i]);
    }
    Serial.print("]");
}

float CosineTheoremAngle(float a, float b, float c)
{
    return acos((a * a + b * b - c * c) / (2 * a * b));
}

float CosineTheoremSide(float a, float b, float phi)
{
    return sqrt(a * a + b * b - 2 * a * b * cos(phi));
}

float QuadEqSolution(float a, float b, float c)
{
    float D = b * b - 4 * a * c;

    return (-b + sqrt(D)) / (2 * a);
}

float rad2deg(float rad)
{
    return rad * (180.0 / M_PI);
}

float deg2rad(float degrees)
{
    return degrees * (M_PI / 180.0);
}

BLA::Matrix<3> crossProduct(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
    BLA::Matrix<3> cross;
    cross(0) = a(1) * b(2) - a(2) * b(1);
    cross(1) = -(a(0) * b(2) - a(2) * b(0));
    cross(2) = a(0) * b(1) - a(1) * b(0);

    return cross;
}