cmake_minimum_required(VERSION 3.10)
project(surface_retruction VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

cmake_policy(SET CMP0072 NEW)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)

set(OrbbecSDK_DIR "/home/adrewn/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

add_subdirectory(Engines)
add_subdirectory(FernRelocLib)

add_executable(collect_data test/collect_data.cpp)

# target_include_directories(collect_data PRIVATE  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(collect_data PRIVATE OrbbecSDK::OrbbecSDK ${OpenCV_LIBS})

add_executable(test_relocaiser test/test_relocaiser.cpp)
target_link_libraries(test_relocaiser PRIVATE FernRelocLib)