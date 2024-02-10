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
//Returns Angle in Rad with sides a,b,c where c is oposite to the angle
float CosineTheoremAngle(float a,float b, float c){
    return acos((a*a+b*b-c*c)/(2*a*b));
}
//Returns Side length in cm with sides a,b, and angle phi in rad
float CosineTheoremSide(float a,float b, float phi){
    return sqrt(a*a+b*b-2*a*b*cos(phi));
}

float QuadEqSolution(float a, float b, float c)
{   
    float D=b*b-4*a*c;

    return (-b+sqrt(D))/(2*a);
}

float rad2angle(float rad)
{
    return rad * (180.0 / M_PI);
}

float angle2rad(float degrees) {
    return degrees * (M_PI / 180.0);
}