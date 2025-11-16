add_library(ViewBuilding src/ViewBuilder.cpp)

target_link_libraries(ViewBuilding PUBLIC ${OpenCV_LIBS})
target_include_directories(ViewBuilding PUBLIC include  ${OpenCV_INCLUDE_DIRS} ../Views/include)