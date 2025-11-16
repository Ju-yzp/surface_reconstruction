#ifndef MAIN_ENGINE_HPP_
#define MAIN_ENGINE_HPP_

#include <opencv2/opencv.hpp>

#include <../../ViewBuilding/include/ViewBuilding/ViewBuilder.hpp>
#include <../../Views/include/Views/View.h>
#include <../../MultiScene/include/MultiScene/ActiveMapManager.hpp>

class MainEngine{
    public:
    void processFrame(cv::Mat rgb_image, cv::Mat depth_image);

    private:
    ViewBuilder *view_builder_;

    View *view_;

    ActiveMapManager *mActiveManager_;
};
#endif