#include <ros/ros.h>

#include <bot_vision/vision.h>
#include <dynamic_reconfigure/server.h>
#include <bot_vision/VisionConfig.h>

// Vision processing params
int thresh_high_h = 130;
int thresh_high_s = 255;
int thresh_high_v = 255;
int thresh_low_h = 80;
int thresh_low_s = 80;
int thresh_low_v = 80;

double fullness_low = 0.10;
double fullness_high = 0.25;

int morph_kernel_size = 5;

int simplify_iterations = 20;

void configure_callback(bot_vision::VisionConfig &config, uint32_t level) {
    // Change parameters
    thresh_high_h = config.thresh_high_h;
    thresh_high_s = config.thresh_high_s;
    thresh_high_v = config.thresh_high_v;
    thresh_low_h = config.thresh_low_h;
    thresh_low_s = config.thresh_low_s;
    thresh_low_v = config.thresh_low_v;

    fullness_low = config.fullness_low;
    fullness_high = config.fullness_high;

    morph_kernel_size = config.morph_kernel_size;

    simplify_iterations = config.simplify_iterations;

    ROS_INFO_STREAM("Parameters have been updated:"
            << "\nthresh_high_h: " << thresh_high_h
            << "\nthresh_high_s: " << thresh_high_s
            << "\nthresh_high_v: " << thresh_high_v
            << "\nthresh_low_h: " << thresh_low_h
            << "\nthresh_low_s: " << thresh_low_s
            << "\nthresh_low_v: " << thresh_low_v
            << "\nfullness_high: " << fullness_high
            << "\nfullness_low: " << fullness_low
            << "\nmorph_kernel_size: " << morph_kernel_size
            << "\nsimplify_iterations: " << simplify_iterations);
}

int main(int argc, char *argv[]) {
    // Init ROS
    ros::init(argc, argv, "bot_vision_node");
    // Get handle to current node
    ros::NodeHandle handle("~");

    // Load parameter values
    handle.param("thresh_high_h", thresh_high_h, thresh_high_h);
    handle.param("thresh_high_s", thresh_high_s, thresh_high_s);
    handle.param("thresh_high_v", thresh_high_v, thresh_high_v);
    handle.param("thresh_low_h", thresh_low_h, thresh_low_h);
    handle.param("thresh_low_s", thresh_low_s, thresh_low_s);
    handle.param("thresh_low_v", thresh_low_v, thresh_low_v);
    handle.param("fullness_high", fullness_high, fullness_high);
    handle.param("fullness_low", fullness_low, fullness_low);
    handle.param("morph_kernel_size", morph_kernel_size, morph_kernel_size);
    handle.param("simplify_iterations", simplify_iterations, simplify_iterations);

    // Set up dynamic_reconfigure
    dynamic_reconfigure::Server<bot_vision::VisionConfig> server;
    server.setCallback(configure_callback);
    
    ros::spin();

    return 0;
}
