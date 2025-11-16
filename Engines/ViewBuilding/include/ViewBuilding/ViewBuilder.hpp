#ifndef VIEW_BUILDER_HPP_
#define VIEW_BUILDER_HPP_

#include <../../Views/include/Views/View.h>

#include <opencv2/opencv.hpp>

class ViewBuilder{
    public:

    void updateView(View **view_ptr, cv::Mat& rgb, cv::Mat& depth, bool useBilateralFilter, bool modelSensorNoise = false, bool storePreviousImage = true);

    private:

    cv::Mat float_img;

    cv::Mat short_img;
};

#endif