// cpp
#include <cstdint>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>
#include "VisualisationEngine/RenderState.h"

// tracker
#include <Tracker/cameraParams.h>
#include <Tracker/pixelUtils.h>

// visualsation
#include <VisualisationEngine/TrackingState.h>
#include <VisualisationEngine/View.h>
#include <VisualisationEngine/VisualisationEngine.h>
#include <VisualisationEngine/VoxelBlockHash.h>
#include <opencv2/core/hal/interface.h>

constexpr float MAX_UINT16_VALUE = static_cast<float>(std::numeric_limits<uint16_t>::max());
constexpr float SCALE_FACTOR = 1000.0f;

uint16_t float_to_uint16(float float_value, float scale_factor = SCALE_FACTOR) {
    float scaled_value = float_value * scale_factor;
    float clamped_value = std::fmax(0.0f, std::fmin(scaled_value, MAX_UINT16_VALUE));
    float rounded_value = std::round(clamped_value);
    return static_cast<uint16_t>(rounded_value);
}

cv::Mat getDepth(std::string file_path) {
    cv::Mat origin = cv::imread(file_path, cv::IMREAD_UNCHANGED);
    cv::Mat convert;
    convertShortToFloat(&origin, &convert, 5000.0f);
    return convert;
}

cv::Mat convertDepth(cv::Mat pointMap) {
    cv::Mat depth;
    depth.create(pointMap.rows, pointMap.cols, CV_16U);
    for (int y{0}; y < pointMap.rows; ++y) {
        for (int x{0}; x < pointMap.cols; ++x) {

            if(pointMap.at<cv::Vec4f>(y, x)(3) >= 0.0f){
              depth.at<uint16_t>(y, x) = float_to_uint16(pointMap.at<cv::Vec4f>(y, x)(2) * 5000.0f);
              std::cout<<pointMap.at<cv::Vec4f>(y, x)(2)<<std::endl;
            }
            else
              depth.at<uint16_t>(y, x) = 0;
        }
    }
    return depth;
}

int main() {
    // 相机参数
    Intrinsic depth(525.0f, 525.0f, 319.5f, 239.5f);
    RGBDCalibrationParams calibrationParams(depth, depth, Eigen::Matrix4f(), 0.3f, 4.0f, 5000.0f);

    // 深度图
    std::string file1_path = "/home/adrewn/surface_restruction/data/1305031102.194330.png";
    std::shared_ptr<View> view = std::make_shared<View>(calibrationParams);
    view->depth = getDepth(file1_path);

    SceneParams sceneParams{0.02f, 100.0f, 0.05f};

    std::shared_ptr<TrackingState> ts =
        std::make_shared<TrackingState>(cv::Size2i(view->depth.cols, view->depth.rows));
    std::shared_ptr<RenderState> rs =
        std::make_shared<RenderState>(cv::Size2i(view->depth.cols, view->depth.rows));
    {
        Eigen::Vector3f translation(1.3352f, 0.6261f, 1.6519f);
        Eigen::Quaternionf rotation_q(-0.3231f, 0.6564f, 0.6139f, -0.2963f);
        rotation_q.normalize();
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = rotation_q.toRotationMatrix();
        T.block<3, 1>(0, 3) = translation;
        ts->pose_d = T;
    }

    std::shared_ptr<Scene> scene = std::make_shared<Scene>(sceneParams);
    scene->reserveVisibleVoxelBlockList(view->depth.rows * view->depth.cols);

    //可视化引擎
    VisualisationEngine ve;
    ve.processFrame(scene, view, ts);
    scene->swapVisibleList();
    ve.prepare(scene, view, ts, rs);

    cv::Mat p = convertDepth(ts->pointsMap);
    cv::namedWindow("Subsample", cv::WINDOW_AUTOSIZE);

    cv::imshow("Subsample", p);
    ;
    cv::waitKey();

    // {
    //     Eigen::Vector3f translation(1.3434, 0.6271, 1.6606);
    //     Eigen::Quaternionf rotation_q(-0.3266, 0.6583, 0.6112, -0.2938);
    //     rotation_q.normalize();
    //     Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    //     T.block<3, 3>(0, 0) = rotation_q.toRotationMatrix();
    //     T.block<3, 1>(0, 3) = translation;
    //     ts->pose_d = T;
    // }
    // file1_path = "/home/adrewn/surface_restruction/data/1305031102.160407.png";
    // view->depth = getDepth(file1_path);
    // ve.processFrame(scene, view, ts);
}
