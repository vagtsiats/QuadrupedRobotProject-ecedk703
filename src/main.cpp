#include <BasicLinearAlgebra.h>
#include <BLA_tools.h>
// #include <trajectory.h>

void setup()
{
    Serial.begin(9600);

    BLA::Matrix<2, 4> final_transform = {0, 0, 0, 0, 1, 1, 1, 1};
    printMatrix(final_transform);
}

void loop()
{
    // Your loop code here
}
