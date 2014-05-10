/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  Modified By NEEC Program Participants: 
  Griffin Jarmin
  Ryan Willard
  James Reed
 
  May 2014
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AQ_PID_H_
#define _AQ_PID_H_

#include <chrono.h>

float currentTime;
bool inFlight;
enum {
  THROTTLE=0,
  PITCH,    
  ROLL,

  LAST_PID_IDX  // keep this definition at the end of this enum
};

//// PID Variables
struct PIDdata {
  float P, I, D;
  float lastError;
  // AKA experiments with PID
  float previousPIDTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
} PID[LAST_PID_IDX];

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
//float windupGuard; // Read in from EEPROM
//// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso

float constrain(float a, float x, float y){
  if(a>x&&a<y) return a;
  if(a<x) return x;
  if(a>y) return y;
}

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (unsigned char axis = 0; axis <= ROLL; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}

float getCurrentTime(){
 std::chrono::steady_clock::time_point tpCurrent=std::chrono::steady_clock::now();
  return std::chrono::duration_cast<microseconds>(tpCurrent).count();

}

float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {

  // AKA PID experiments
  //TODO::set current time equal to a current time in ms
  
  currentTime=getCurrentTime();
  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  float error = targetPosition - currentPosition;

  if (inFlight) {
    PIDparameters->integratedError += error * deltaPIDTime;
  }
  else {
    PIDparameters->integratedError = 0.0;
  }
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  float dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastError) / (deltaPIDTime * 100); // dT fix from Honk
  PIDparameters->lastError = currentPosition;

  return (PIDparameters->P * error) + (PIDparameters->I * PIDparameters->integratedError); //This does not include the dTerm

}

void initPID(){
 PID[THROTTLE].P=1;
 PID[THROTTLE].I=1;
 PID[THROTTLE].D=1;
 PID[THROTTLE].windupGuard=100;
 PID[THROTTLE].previousPIDTime=getCurrentTime();


 PID[ROLL].P=1;
 PID[ROLL].I=1;
 PID[ROLL].D=1;
 PID[ROLL].windupGuard=100;
 PID[ROLL].previoudPIDTime=getCurrentTime();


 PID[PITCH].P=1;
 PID[PITCH].I=1;
 PID[PITCH].D=1;
 PID[PITCH].windupGuard=100;
 PID[PITCH].previoudPIDTime=getCurrentTime();

}




#endif // _AQ_PID_H_

