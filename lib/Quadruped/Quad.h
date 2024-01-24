#include "Leg.h"
#include "BLA_tools.h"
class Quad
{

public:
    Quad(/* args */);
    ~Quad();
   void initHardware();
   void inverseDiffKinematics(unsigned int time0);

   private:
   Leg br;
   Leg bl;
   Leg fr;
   Leg fl;

};

