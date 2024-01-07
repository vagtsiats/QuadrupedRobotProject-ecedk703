#include "Leg.h"
Leg::Leg()
{
    shoulder.attach(9);
    knee.attach(9);
    ankle.attach(9);

}

Leg::~Leg()
{
}
void Leg::DriveLeg(int up,int mid ,int low){

    shoulder.write(up);
    knee.write(mid);
    ankle.write(low);

}
