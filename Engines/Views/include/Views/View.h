#ifndef VIEW_H_
#define VIEW_H_

#include <opencv2/opencv.hpp>

class View{
    public:
    cv::Mat rgb;
    cv::Mat rgb_prev;
    cv::Mat depth;
    cv::Mat depth_normal;
    cv::Mat depth_uncertainty;
    cv::Mat depth_confidence;
};

#endif