#include <BLA_tools.h>

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

float rad2deg(float rad)
{
    return rad * (180.0 / M_PI);
}

float deg2rad(float degrees) {
    return degrees * (M_PI / 180.0);
}