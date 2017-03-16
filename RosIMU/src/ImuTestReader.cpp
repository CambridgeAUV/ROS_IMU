#include "ros/ros.h"
#include "std_msgs/String.h"

void imu_dataCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("IMU Reader read: [%s]", msg->data.c_str());	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImuTestReader");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("imu_data", 1000, imu_dataCallback);
	ros::spin();
	return 0;
	
}