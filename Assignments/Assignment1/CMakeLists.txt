cmake_minimum_required(VERSION 2.8.12)

project(lk-tracker)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_library(tracker-cc src/helpers.cc)

add_executable(lk-tracking scripts/lk-tracker.cpp)
target_link_libraries(lk-tracking ${OpenCV_LIBS})
target_link_libraries(lk-tracking tracker-cc)
