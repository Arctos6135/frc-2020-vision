#pragma once

#include <opencv2/opencv.hpp>

namespace vision {
    void preprocess(cv::InputArray src, cv::OutputArray dst, cv::Scalar thresh_low, cv::Scalar thresh_high, int kern_size, bool morph = true);
}
