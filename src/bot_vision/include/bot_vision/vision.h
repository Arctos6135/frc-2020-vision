#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace vision {
    void preprocess(const cv::Mat &src, cv::Mat &dst, cv::Scalar thresh_low, cv::Scalar thresh_high, int kern_size);
    int filter_targets(const std::vector<std::vector<cv::Point> > &contours, double fullness_low, double fullness_high);
    bool simplify_contour(const std::vector<cv::Point> &contour, std::vector<cv::Point> &out, int n, int max_iterations);
    bool find_target_contour(const cv::Mat &img, std::vector<cv::Point> &out, double fullness_low, double fullness_high, int simplify_iterations);
}
