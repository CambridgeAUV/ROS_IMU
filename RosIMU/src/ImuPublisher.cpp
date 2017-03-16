#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//
int main(int argc, char** argv)
{
	//Andrew adding ros stuff
	ros::init(argc, argv, "ImuPublisher");
	ros::NodeHandle n;
	//The ros topic for the imu data is called 'imu_data'
	ros::Publisher imu_pub = n.advertise<std_msgs::String>("imu_data", 1000);


	SbgProtocolHandle protocolHandle;
	SbgErrorCode error;
	SbgOutput output;

	//
	// Init our communications with the device (Please change here the com port and baud rate)
	//
	if (sbgComInit("/dev/ttyUSB0", 115200, &protocolHandle) == SBG_NO_ERROR)
	{
		//
		// Wait until the device has been initialised
		//
		sbgSleep(50);
		
		//
		// Display the title
		//
		printf("Euler Angles:\n");

		//
		// Main loop
		//
		int test_counter = 0; //This is just a test counter for testing the publishing functionality
		while (1)
		{
			error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
			if (error == SBG_NO_ERROR)
			{
				//
				// Displays sensor values in the console
				//
				printf("%3.2f\t%3.2f\t%3.2f\n",	SBG_RAD_TO_DEG(output.stateEuler[0]),
												SBG_RAD_TO_DEG(output.stateEuler[1]),
												SBG_RAD_TO_DEG(output.stateEuler[2]));
				std_msgs::String msg;
				std::stringstream ss;
				ss << SBG_RAD_TO_DEG(output.stateEuler[0]) << " " << SBG_RAD_TO_DEG(output.stateEuler[1]) << " " << SBG_RAD_TO_DEG(output.stateEuler[2]) ;
				//test_counter = test_counter + 1;
				msg.data = ss.str();
				
				imu_pub.publish(msg);
				ros::spinOnce();
			}

			//
			// Small pause to unload CPU
			//
			sbgSleep(10);
		}

		//
		// Close our protocol system
		//
		sbgProtocolClose(protocolHandle);

		return 0;
	}
	else
	{
		fprintf(stderr, "Unable to open IG-500 device\n");
		return -1;
	}
}
