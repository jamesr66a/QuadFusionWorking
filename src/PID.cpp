
#include "PID.h"
#include <chrono>


PID::PID()
{
	P = 1;
	I = 1;
	windupGuard = 100;
	currentTime = getCurrentTime();
	previousPIDTime = getCurrentTime();
}

void PID::PID(float P, float I, float windupGuard){
	this->P=P;
	this->I=I
	this->windupGuard=windupGuard;
	currentTime=getCurrentTime();
	previousPIDTime=getCurrentTime();
	
}

float PID::updatePID(float targetPosition, float currentPosition)
{
	currentTime = getCurrentTime();
	const float deltaPIDTime = (currentTime - previousPIDTime) / 1000000.0;

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

float getCurrentTime()
{
	auto tpCurrent = std::chrono::high_resolution_clock::now();

	return std::chrono::milliseconds(tpCurrent).count();
}
