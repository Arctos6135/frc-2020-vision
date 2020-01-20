#include <ros/ros.h>
#include "bot_vision/vision.h"
#include <iostream>

int main(int argc, char *argv[]) {
    // Init ROS
    ros::init(argc, argv, "bot_vision_node");
    // Get handle to current node
    ros::NodeHandle handle("~");
    std::cout << "Hello world" << std::endl;
    ros::spin();

    return 0;
}
