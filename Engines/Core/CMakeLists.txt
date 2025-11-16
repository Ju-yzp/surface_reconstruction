add_library(Core src/MainEngine.cpp)

target_link_libraries(Core PUBLIC ${OpenCV_LIBS})
target_include_directories(Core PUBLIC include  ${OpenCV_INCLUDE_DIRS})