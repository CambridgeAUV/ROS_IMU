# ROS_IMU_arc82

THIS DOES NOT DO INTEGRATIONS. THAT IS IN A SEPARATE REPO
You will first need to be able to run the IMU code 'ig500Minimal' without ROS before using this.

You need to change the line "link_directories(/home/andrew/Documents/CAUV/LibFolder)" in RosIMU/CMakeLists.txt to a folder on your machine that contains the sbg library called "libSbgComSerial.a"
