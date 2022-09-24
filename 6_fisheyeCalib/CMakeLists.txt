cmake_minimum_required(VERSION 2.8)

project(fisheyeCalib)

set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(run_fisheyeCalib ${PROJECT_SOURCE_DIR}/test/fisheyeCalib.cpp)

target_link_libraries(run_fisheyeCalib ${OpenCV_LIBS})
