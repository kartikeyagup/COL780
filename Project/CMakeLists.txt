cmake_minimum_required(VERSION 2.8.12)

project(video-stitch)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(view-morpher scripts/ViewMorpher.cpp)
target_link_libraries(view-morpher ${OpenCV_LIBS})
