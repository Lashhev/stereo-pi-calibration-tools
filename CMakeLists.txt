cmake_minimum_required(VERSION 3.10)
project(stereo_calibrator)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -funsafe-math-optimizations -O3 -pipe")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)

add_subdirectory(modules/mono-camera-calibrator)

add_library(${PROJECT_NAME} SHARED src/stereo-camera-calibrator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} mono_calibrator ${OpenCV_LIBS})

