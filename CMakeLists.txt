cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")

project(FaceDetectionSerial VERSION 0.1)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(dlib dlib_build)

add_executable(FaceDetectionSerial main.cpp)

target_link_libraries(FaceDetectionSerial ${OpenCV_LIBS} dlib::dlib)
