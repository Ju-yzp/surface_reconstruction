add_library(MutilScene src/ActiveMapManager.cpp)

target_include_directories(MutilScene PUBLIC
    ${PROJECT_SOURCE_DIR}/Engines/MutliScene/include
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)


target_link_libraries(MutilScene PUBLIC
    ${OpenCV_LIBS}
    ${G2O_LIBRARIES}
)