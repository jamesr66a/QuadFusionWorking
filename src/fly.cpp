#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>


#include "optical_flow.h"
#include "cameraPoseEstimator.h"
#include "PID.h"
#include "GPIO.h"


int constrain(int a, int x, int y){
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
    Pitch.setP(15);
    Roll.setP(15);
    Pitch.setI(10);
    Roll.setI(10);
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
		Pitch.zeroIntegralError();
		Roll.zeroIntegralError();
		Pitch.setPwmOut(15000);
		Roll.setPwmOut(15000);
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
	 rollError=Roll.updatePID(setPointY, -y, inFlight);
	 throttleError=Throttle.updatePID(setPointZ, z, inFlight);
	 
	 //Pitch.setPwmOut(constrain((Pitch.getPwmOut()+(pitchError*100)),14000,15000));
	 //Roll.setPwmOut(constrain((Roll.getPwmOut()+(rollError*100)),14000,15000));
	 //Throttle.setPwmOut(constrain((Throttle.getPwmOut()+(throttleError*100)),1000,2000));
	 //PW8<<Throttle.getPwmOut();
	 
	 PWM9<<15000-Roll.getPwmOut() << std::flush;
	 PWM10<<15000+Pitch.getPwmOut() << std::flush;
	 
	 
	 std::cout<<"Roll: "<<15121-Roll.getPwmOut()<<"\t";
	 std::cout<<"Pitch: "<<15121+Pitch.getPwmOut()<<"\t";
	 if(inFlight) std::cout<<"Switch Flipped";
	 std::cout<<std::endl;
	
	//std::cout <<"X: "<< x <<" cm"<< " Y:" << y << " cm Z: " << z <<" m" << std::ends;
	//std::cout << std::setw(10) << pitchError << " " << std::setw(10) << rollError << " " << std::setw(10) << throttleError << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
