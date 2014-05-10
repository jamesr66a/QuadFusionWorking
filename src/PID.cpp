
#include "PID.h"
#include <chrono>
#include <iostream>


PID::PID()
{
	P = 1;
	I = 1;
	windupGuard = 100;
	currentTime = std::chrono::high_resolution_clock::now();
	previousPIDTime = currentTime;
}

void PID::setI(float I){
	this->I=I;
}

void PID::setP(float P){
	this->P=P;
}

void PID::setWindupGuard(float windupGuard){
	this->windupGuard=windupGuard;
}

float PID::updatePID(float targetPosition, float currentPosition, bool inFlight)
{
	currentTime = std::chrono::high_resolution_clock::now();
	//std::chrono::time_point deltaPIDTime = (currentTime - previousPIDTime);
	
	float deltaPIDTime=(std::chrono::duration_cast<std::chrono::microseconds>(currentTime-previousPIDTime).count())/100000.0;
	
	std::cout<<deltaPIDTime<<std::endl;

	previousPIDTime = currentTime;
	float error = targetPosition - currentPosition;

	if(inFlight)
	{
		integratedError += error * deltaPIDTime;
	}
	else
	{
		integratedError = 0.0;
	}

	integratedError = constrain(integratedError, -windupGuard, windupGuard);

	return P * error + I * integratedError;
}

float PID::constrain(float a, float x, float y)
{
	if(a < x)
	{
		return x;
	}
	else if(a > y)
	{
		return y;
	}
	else
	{
		return a;
	}
}

void PID::zeroIntegralError()
{
	integratedError = 0;
	previousPIDTime = currentTime;
}

