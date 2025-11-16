#include <Core/MainEngine.hpp>

struct ToDoListEntry{
    ToDoListEntry(int activeDataID, bool track_, bool fusion_, bool prepare_)
    :data_id(activeDataID), track(track_), fusion(fusion_), prepare(prepare_),preprepare(prepare_)
    {}

    int data_id;
    bool track;
    bool fusion;
    bool prepare;
    bool preprepare;
};

void MainEngine::processFrame(cv::Mat rgb_image, cv::Mat depth_image){

    std::vector<ToDoListEntry> todoList;
    view_builder_->updateView(&view_, rgb_image, depth_image, true);

    int primary_data_id = mActiveManager_->findPrimaryDataIdx();

    if(primary_data_id > 0)
      todoList.emplace_back(ToDoListEntry(primary_data_id, true, true, true));

    for(int i{0}; i < mActiveManager_->numActiveLocalMaps(); ++i){
        switch (mActiveManager_->getLocalMapType(i)) {
            case LocalMapActivity::NEW_LOCAL_MAP:
            todoList.emplace_back(ToDoListEntry(i, true, true, true));
            case LocalMapActivity::LOOP_CLOSURE:
            todoList.emplace_back(ToDoListEntry(i, true, false, true));
            case LocalMapActivity::RELOCALISATION:
            todoList.emplace_back(ToDoListEntry(i, true, false, true));
            default:
            break;
        }
    }

    todoList.emplace_back(ToDoListEntry(-1,false,true,false));

    bool primary_tracking_sucess{false};

    for(int i{0}; i < todoList.size(); ++i){
        if(todoList[i].data_id == -1){
            
        }
    }
}