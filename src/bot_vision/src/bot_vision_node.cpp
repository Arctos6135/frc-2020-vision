#include <ros/ros.h>

#include "bot_vision/vision.h"
#include <dynamic_reconfigure/server.h>
#include "bot_vision/VisionConfig.h"

void configure_callback(bot_vision::VisionConfig &config, uint32_t level) {
    
}

int main(int argc, char *argv[]) {
    // Init ROS
    ros::init(argc, argv, "bot_vision_node");
    // Get handle to current node
    ros::NodeHandle handle("~");

    // Set up dynamic_reconfigure
    dynamic_reconfigure::Server<bot_vision::VisionConfig> server;
    server.setCallback(configure_callback);
    
    ros::spin();

    return 0;
}
