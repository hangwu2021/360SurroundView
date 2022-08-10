cmake_minimum_required(VERSION 3.0)

project(imageBasics)

# OpenCV 
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main 2-1-2-UMat.cpp)

target_link_libraries(main ${OpenCV_LIBS})
