#include "bot_vision/vision.h"

namespace vision {
    void preprocess(cv::InputArray src, cv::OutputArray dst, cv::Scalar thresh_low, cv::Scalar thresh_high, int kern_size, bool morph = true) {
        // Convert image to HSV colour space
        // FULL means a hue of 0-360 degrees
        cv::cvtColor(src, dst, cv::COLOR_BGR2HSV_FULL);
        // Threshold
        cv::inRange(dst, thresh_low, thresh_high, dst);
        // Apply morphological operations
        if(morph) {
            cv::Mat kern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kern_size, kern_size));
            cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kern);
            cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kern);
        }
    }
}
