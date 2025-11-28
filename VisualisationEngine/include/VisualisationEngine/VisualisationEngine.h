#ifndef VISUALISATION_ENGINE_H_
#define VISUALISATION_ENGINE_H_

// cpp
#include <memory>

// VisualisationEngine
#include <VisualisationEngine/RenderState.h>
#include <VisualisationEngine/TrackingState.h>
#include <VisualisationEngine/View.h>
#include <VisualisationEngine/VoxelBlockHash.h>

class VisualisationEngine {
public:
    void processFrame(
        std::shared_ptr<Scene> scene, std::shared_ptr<View> view,
        std::shared_ptr<TrackingState> trackingState);

    void prepare(
        std::shared_ptr<Scene> scene, std::shared_ptr<View> view,
        std::shared_ptr<TrackingState> trackingState, std::shared_ptr<RenderState> renderState);

private:
    void allocateMemoryFromDepth(
        std::shared_ptr<Scene> scene, std::shared_ptr<View> view,
        std::shared_ptr<TrackingState> trackingState);

    void integrateIntoScene(
        std::shared_ptr<Scene> scene, std::shared_ptr<View> view,
        std::shared_ptr<TrackingState> trackingState);

    void raycast(
        std::shared_ptr<Scene> scene, std::shared_ptr<View> view,
        std::shared_ptr<TrackingState> trackingState, std::shared_ptr<RenderState> renderState);

    void produceNeedICP(
        Eigen::Vector3f lightSource, float voxelSize, std::shared_ptr<RenderState> renderState,
        std::shared_ptr<TrackingState> trackingState);
};

#endif
