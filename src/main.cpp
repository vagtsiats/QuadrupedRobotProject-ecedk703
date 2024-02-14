#include "definitions.h"
#include <cstring> 
using namespace BLA;
#define HWSERIAL Serial1

//SECTION Wiring
// Connect pin 0(Rx pin of MCU) of Arduino with TX pin of ESP
//Common GND
void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);
}
void loop() {

    if (HWSERIAL.available() > 0) {
        processIncomingByte (HWSERIAL.read());
    }
}
