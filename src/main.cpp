#include "definitions.h"
using namespace BLA;

void setup()
{
    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, 1);

    Robot.initHardware();
}

void loop()
{
    BLA::Matrix<3> q = {0.9, 0.2, -0.5};

    Robot.br.update_leg(q);

    BLAprintMatrix(Robot.br.getJacobian());
    Serial.println();
}
