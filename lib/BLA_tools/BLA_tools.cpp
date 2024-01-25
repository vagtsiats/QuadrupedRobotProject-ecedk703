#include <BLA_tools.h>

void printvector(const std::vector<double> &a)
{
    Serial.print("[");
    for (int i = 0; i < a.size(); i++)
    {
        Serial.print(a[i]);
        Serial.print(",");
    }
    Serial.print("]");
}
