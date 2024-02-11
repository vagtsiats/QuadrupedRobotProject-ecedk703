#include "Turn.h"

Turn::Turn(float t_vd, float t_y0) : vd(t_vd), y0(t_y0)
{
}
void Turn::calculateTurnParameters()
{
    float a=atan(l/n);
    float m=n/(2*cos(a));
    d=CosineTheoremSide(m,m,phi);
    x=QuadEqSolution(2*(1+cos(phi)),-2*l*(1+cos(phi)),l*l-d*d);
    y=QuadEqSolution(2*(1+cos(phi)),-2*n*(1+cos(phi)),n*n-d*d);
    theta1=asin(sin(phi)*(l-x)/d);
    theta2=asin(sin(phi)*(n-y)/d);
    
    Serial.println(d);
    Serial.println(rad2deg(theta1));
    Serial.println(rad2deg(theta2));
}

