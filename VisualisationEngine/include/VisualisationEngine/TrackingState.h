#ifndef TRACKING_STATE_H_
#define TRACKING_STATE_H_

// eigen
#include <Eigen/Eigen>

// opencv
#include <opencv2/opencv.hpp>

struct TrackingState {
    Eigen::Matrix4f pose_d;

    enum class TrackingResult { TRACKING_GOOD = 0, TRACKING_POOR = 1, TRACKING_FAILED = 2 };

    TrackingResult trackingResult;

    // 法向量图
    cv::Mat normalMap;

    // 点云图
    cv::Mat pointsMap;

    TrackingState(cv::Size2i imgSize) {
        normalMap = cv::Mat(imgSize.height, imgSize.width, CV_32FC4);
        pointsMap = cv::Mat(imgSize.height, imgSize.width, CV_32FC4);
    }
};

#endif
