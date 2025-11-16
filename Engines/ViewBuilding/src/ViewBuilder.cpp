#include <ViewBuilding/ViewBuilder.hpp>
#include <opencv2/opencv.hpp>

void ViewBuilder::updateView(View **view_ptr, cv::Mat& rgb, cv::Mat& depth, bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage){

    View *view = *(view_ptr);

    view->rgb = rgb;

    if(useBilateralFilter){
        cv::bilateralFilter(this->float_img, view->depth, 9.0, 75.0, 75.0);
        cv::bilateralFilter(view->depth, this->float_img, 9.0, 75.0, 75.0);
        cv::bilateralFilter(this->float_img, view->depth, 9.0, 75.0, 75.0);
        cv::bilateralFilter(view->depth, this->float_img, 9.0, 75.0, 75.0);
        cv::bilateralFilter(this->float_img, view->depth, 9.0, 75.0, 75.0);
    }
}