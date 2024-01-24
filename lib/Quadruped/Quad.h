#include "Leg.h"
#include "BLA_tools.h"
class Quad
{

public:
    Quad(/* args */);
    ~Quad();
   void initHardware();
    Leg br;

   private:
   Leg bl;
   Leg fr;
   Leg fl;

};

