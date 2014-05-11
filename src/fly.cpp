#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>


#include "optical_flow.h"
#include "cameraPoseEstimator.h"
#include "PID.h"
#include "GPIO.h"


int main()
{
    CameraPoseEstimator cpe;
    OpticalFlowSensor ofs;

    initGPIO(10, true);

    std::thread cpe_thread(&CameraPoseEstimator::continuousRead, &cpe);
    std::thread ofs_thread(&OpticalFlowSensor::loop, &ofs, std::string("/dev/ttyO0"));
	
    float x = 0, y = 0, z = 0;
    float rollError=0, pitchError=0, throttleError=0;
    
    PID Pitch, Roll, Throttle;
    Throttle.setPwmOut(10000);
    float setPointX=0, setPointY=0, setPointZ=0;
    bool inFlight = false, prevFlightStatus=false, firstRead=true;
    std::ofstream PWM8("/dev/pwm8"); //Throttle
    std::ofstream PWM9("/dev/pwm9"); //Roll
    std::ofstream PWM10("/dev/pwm10"); //Pitch
    std::ofstream PWM11("/dev/pwm11"); //Yaw
    //float pidRoll, pidPitch, pidThrottle;

    
    //initPID();

    //initGPIO(16,false);
    //writeGPIO(16, true);
    //initGPIO(17,true);

    while (true)
    {
        if(readGPIO(10)=='1') inFlight=true;
        if(readGPIO(10)=='0') inFlight=false;
        
	if(inFlight == true && prevFlightStatus == false)
	{
		setPointX=x;
		setPointY=y;
		setPointZ = z;
	}
	
	prevFlightStatus=inFlight;
	

	FlowData fd;
	Pose3D pose;
	
        if (ofs.dataReady())
	{
	    fd = ofs.getFlowData();
	    //std::cout << fd.flow_x << " "
	    //	      << fd.flow_y << " "
	    //	      << std::endl;
	    x += fd.flow_x;
	    y += fd.flow_y;
	    z = fd.ground_distance;
	   	    //std::cout << std::chrono::nanoseconds(dt).count()/10e6 << std::endl;
	}
	if (cpe.dataAvailable())
	{
	    cpe.getPose(pose);
	
	    x = pose.x;
   	    y = pose.y;
	    
            //if(firstRead){
		
	    //    currentTime=getCurrentTime();

		//firstRead=false;

//	    }
	    //pidPitch=updatePID(setPointX,x,&PID[PITCH]);
            //pidRoll=updatePID(setPointY,y,&PID[ROLL]);

	    /*TODO: get PID for throttle and then do
	     1500+ constrain stuff. 
	     figure it out James, I'm tired now.

		-Griffin

	    */	   
	}
	
	 pitchError=Pitch.updatePID(setPointX, x, inFlight);
	 rollError=Roll.updatePID(setPointY, y, inFlight);
	 throttleError=Throttle.updatePID(setPointZ, z, inFlight);
	 
	 Pitch.setPwmOut(Pitch.getPwmOut()+(pitchError*100));
	 Roll.setPwmOut(Roll.getPwmOut()+(rollError*100));
	 //Throttle.setPwmOut(Throttle.getPwmOut()+(throttleError*100));
	 //PW8<<Throttle.getPwmOut();
	 PWM9<<Roll.getPwmOut();
	 PWM10<<Pitch.getPwmOut();
	 
	
	std::cout <<"X: "<< x <<" cm"<< " Y:" << y << " cm Z: " << z <<" m" << std::ends;
	std::cout << std::setw(10) << pitchError << " " << std::setw(10) << rollError << " " << std::setw(10) << throttleError << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
