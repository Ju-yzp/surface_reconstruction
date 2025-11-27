#ifndef TRACKING_STATE_H_
#define TRACKING_STATE_H_

#include <Eigen/Eigen>

struct TrackingState {
    Eigen::Matrix4f pose_d;

    enum class TrackingResult { TRACKING_GOOD = 0, TRACKING_POOR = 1, TRACKING_FAILED = 2 };

    TrackingResult trackingResult;
};

#endif
