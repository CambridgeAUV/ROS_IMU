#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <msg_definitions/imu_data.h> //This is the custom message file. It is stored in another package within the same repository called msg_definitions
#include <iostream>

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//
int main(int argc, char** argv)
{
	//Andrew adding ros stuff
	ros::init(argc, argv, "ImuPublisher");
	ros::NodeHandle n;
	//The ros topic for the imu data is called 'imu_data'
	ros::Publisher imu_pub = n.advertise<msg_definitions::imu_data>("imu_data", 1000);


	SbgProtocolHandle protocolHandle;
	SbgErrorCode error;
	SbgOutput output;
	msg_definitions::imu_data msg_packet; 

	//
	// Init our communications with the device (Please change here the com port and baud rate)
	//
	if (sbgComInit("/dev/ttyUSB0", 115200, &protocolHandle) == SBG_NO_ERROR)
	{
		//
		// Wait until the device has been initialised
		//
		sbgSleep(50);
		
                if(sbgRestoreDefaultSettings(protocolHandle, false) != SBG_NO_ERROR) {
                    std::cout << "Error restoring defaults" << std::endl;
                }
		//
		// Display the title
		//
		printf("Kalman Positions:\n");

		//
		// Main loop
		//
		
		//while (1)
                for(int i = 0; i < 50; ++i)
		{
                        error = sbgGetSpecificOutput(protocolHandle, 
                            SBG_OUTPUT_EULER | 
                            SBG_OUTPUT_POSITION | 
                            SBG_OUTPUT_VELOCITY | 
                            SBG_OUTPUT_ATTITUDE_ACCURACY |
                            SBG_OUTPUT_NAV_ACCURACY | 
                            SBG_OUTPUT_ACCELEROMETERS |
                            SBG_OUTPUT_GPS_POSITION,
                             &output);
			if (error == SBG_NO_ERROR)
			{
				//
				// Displays sensor values in the console
				//
                               std::cout << "Positions: " << output.position[0] << " " << output.position[1] << " " << output.position[2] <<  std::endl;
                               // std::cout << "Velocities: " << output.velocity[0] << " " << output.velocity[1] << " " << output.velocity[2] << std::endl;
                               // std::cout << "Attitude Accuracy: " << output.attitudeAccuracy << std::endl;
                               // std::cout << "Position accuracy: " << output.positionAccuracy << std::endl;
                               //std::cout << "Accelerations: " << output.accelerometers[0] << " " << output.accelerometers[1] << " " << output.accelerometers[2] << std::endl;
                                //std::cout << "GPS latitude: " << output.gpsLatitude << " longitude: " << output.gpsLongitude << std::endl;
				imu_pub.publish(msg_packet);
				ros::spinOnce();
			}

			//
			// Small pause to unload CPU
			//
			sbgSleep(10);
		}

                char error_log[SBG_ERROR_LOG_SIZE_BYTES];
                if (sbgGetErrorLog(protocolHandle, &error_log, 0) != SBG_NO_ERROR) {
                    std::cout << "Getting error log had an error" << std::endl;
                }
                std::cout << (int)error_log[0] << std::endl;

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
