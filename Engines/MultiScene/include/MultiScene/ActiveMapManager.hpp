#ifndef ACTIVE_MAP_MANAGER_HPP_
#define ACTIVE_MAP_MANAGER_HPP_

#include <eigen3/Eigen/Eigen>
#include <vector>

enum class LocalMapActivity{
    PRIMARY_LOCAL_MAP = 0, 
    NEW_LOCAL_MAP = 1, 
    LOOP_CLOSURE = 2, 
    RELOCALISATION = 3, 
    LOST = 4, 
    LOST_NEW = 5
};

class ActiveMapManager{
    public:


    int initiateNewLocalMap(bool isPrimaryLocalMap = false);

    int initiateNewLink(int sceneID, const Eigen::Isometry3f& pose, bool isRelocalisation);

    // void recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess);
    
    bool maintainActiveData(void);

    int findPrimaryDataIdx(void) const;
    int findPrimaryLocalMapIdx(void) const;

    int findBestVisualisationDataIdx(void) const;
    int findBestVisualisationLocalMapIdx(void) const;

    int numActiveLocalMaps(void) const { return static_cast<int>(activeData_.size()); }

    int getLocalMapIndex(int dataIdx) const { return activeData_[dataIdx].localMapIndex; }

    LocalMapActivity getLocalMapType(int dataIdx) const { return activeData_[dataIdx].type; }

    private:


    int checkSuccess_relocalisation(int dataID) const;

    int checkSuccess_newlink(int dataID, int primaryDataID, int *inliers,  Eigen::Isometry3f& inlierPose) const;

    void acceptNewLink(int dataId, int primaryDataId, const Eigen::Isometry3f& pose, int weight);

    float visibleOriginalBlocks(int dataID) const;

    bool shouldStartNewArea(void) const;

    bool shouldMovePrimaryLocalMap(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

    struct ActiveDataDescriptor 
    {
        int localMapIndex;
        LocalMapActivity type;
        std::vector<Eigen::Matrix4f> constraints;
        Eigen::Isometry3f estimatedPose;
        int trackingAttempts;
    };

    std::vector<ActiveDataDescriptor> activeData_;
};

#endif