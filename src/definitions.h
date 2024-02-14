#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "Leg.h"
#include "BLA_tools.h"
#include "trajectory.h"
#include "Quad.h"
#include "Turn.h"
using namespace BLA;

const u_int32_t LOOP_PERIOD = 1e4; // 0.01sec
u_int32_t timer0;                  // constant loop timer
double timer1;                     // general purpose timer
#define MAX_INPUT 10

Quad Robot;

void processData (const char * data)
  {
    //if first element of data is digit or -
    if(isdigit(*data) || data[0]=='-'){
        float walkSpeed = std::stof(data);
        Serial.println(walkSpeed);
    }
    else if (*data=='r' || *data=='R') {
        //Action for turning right 
        Serial.println("Turn Right");
    }
    else if (*data=='l' || *data=='L') {
        //Action for turning left 
        Serial.println("Turn left");
    }
    else if (*data=='s' || *data=='S') {
        //Action for Stand
        Serial.println("Stand");
    }
    else{
        Serial.println("Non Valid Action");
    }
  }  
  
void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  if(inByte=='\n'){
    input_line [input_pos] = 0;  
        
        // '\n' -> End of line
        process_data (input_line);
        
        // reset buffer for next time
        input_pos = 0; 
  }  
  else{
    if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
  }

}  
// Matrix<3> theta0 = {0, M_PI/6,-M_PI/4};
// float t;

// NOTE - DiffKinematics Test
//  Matrix<3> theta0 = {0, -M_PI / 4, -M_PI / 4};

// Matrix<3> xd(float t)
// {
//     Matrix<3> xd = {7.95, 0, -9.95 + 1 * t};
//     return xd;
// }
// Matrix<3> xd_dot(float t)
// {
//     Matrix<3> xd_dot = {0, 0, 1};
//     return xd_dot;
// }
