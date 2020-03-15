cmake_minimum_required(VERSION 3.15)
project(MySLAM)

set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include(cmake_modules/FindOpenCV.cmake)

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(odometry)

set(EXE_DEMO_NAME "demo_vo_homo")
add_executable(${EXE_DEMO_NAME} demo/demo_vo_homo.cpp)
target_link_libraries(${EXE_DEMO_NAME} odometry)
