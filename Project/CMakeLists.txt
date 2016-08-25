cmake_minimum_required(VERSION 2.8.12)

project(video-stitch)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_library(view-morphing-cc src/viewmorpher.cc
                             src/helpers.cc)

add_executable(view-morphing scripts/ViewMorphing.cpp)
target_link_libraries(view-morphing ${OpenCV_LIBS})
target_link_libraries(view-morphing view-morphing-cc)