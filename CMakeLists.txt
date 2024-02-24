cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

project(FaceDetectionSerial VERSION 0.1)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(dlib dlib_build)

add_executable(FaceDetectionSerial main.cpp)

target_link_libraries(FaceDetectionSerial ${OpenCV_LIBS} dlib::dlib)