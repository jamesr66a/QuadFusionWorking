#include <thread>
#include <chrono>
#include <iostream>


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
    
    PID Pitch, Roll, Throttle;
    float setPointX=0, setPointY=0, setPointZ=0;
    bool inFlight = false, prevFlightStatus=false, firstRead=true;
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
	
	//std::cout <<"X: "<< x <<" cm"<< " Y:" << y << " cm Z: " << z <<" m" << std::ends;
	std::cout << std::setprecision(10) << Pitch.updatePID(setPointX, x, inFlight) << std::setprecision(10) << Roll.updatePID(setPointY, y, inFlight) << std::setprecision(10) << Throttle.updatePID(setPointZ, z, inFlight) << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
