#include <bot_vision/vision.h>

namespace vision {
    void preprocess(const cv::Mat &src, cv::Mat &dst, cv::Scalar thresh_low, cv::Scalar thresh_high, int kern_size) {
        // Convert image to HSV colour space
        // FULL means a hue of 0-360 degrees
        cv::cvtColor(src, dst, cv::COLOR_BGR2HSV_FULL);
        // Threshold
        cv::inRange(dst, thresh_low, thresh_high, dst);
        // Apply morphological operations
        if(kern_size > 0) {
            cv::Mat kern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kern_size, kern_size));
            cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kern);
            cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kern);
        }
    }

    int filter_targets(const std::vector<std::vector<cv::Point>> &contours, double fullness_low, double fullness_high) {
        int max_size = -1;
        int max_index = -1;

        for(int i = 0; i < contours.size(); i ++) {
            double area = cv::contourArea(contours[i]);
            cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            // Filter targets by fullness
            // Fullness is the ratio between the actual area and its bounding box
            double fullness = area / rect.size.area();
            // Pick the one with the largest area that meets fullness requirements
            if(fullness >= fullness_low && fullness <= fullness_high) {
                if(area > max_size) {
                    max_size = area;
                    max_index = i;
                }
            }
        }

        return max_index;
    }

    bool simplify_contour(const std::vector<cv::Point> &contour, std::vector<cv::Point> &out, int n, int max_iterations) {
        double mineps = 0;
        double maxeps = cv::arcLength(contour, true);
        int iterations = 0;

        bool success = false;

        // Do a binary-search like operation on the epsilon value
        // This is to lower the epsilon as much as possible
        while(1) {
            if(iterations >= max_iterations) {
                return success;
            }

            double eps = (mineps + maxeps) / 2;
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, eps, true);
            
            if(approx.size() > n) {
                mineps = eps;
            }
            else if(approx.size() < n) {
                maxeps = eps;
            }
            else {
                maxeps = eps;
                success = true;
                out = std::move(approx);
            }

            iterations ++;
        }
    }

    bool find_target_contour(const cv::Mat &img, std::vector<cv::Point> &out, double fullness_low, 
            double fullness_high, int simplify_iterations) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        int index = filter_targets(contours, fullness_low, fullness_high);

        if(index == -1) {
            return false;
        }

        return simplify_contour(contours[index], out, 8, simplify_iterations);
    }
}
