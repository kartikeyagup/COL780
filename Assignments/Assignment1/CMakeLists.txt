cmake_minimum_required(VERSION 2.8.12)

project(lk-tracker)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lk-tracker scripts/lk-tracker.cpp)
target_link_libraries(lk-tracker ${OpenCV_LIBS})