cmake_minimum_required(VERSION 3.15)
project(MySLAM)

set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR})

include(cmake_modules/FindOpenCV.cmake)

add_subdirectory(odometry)
set(DEMO_LIBS ${DEMO_LIBS} ${THIRD_LIBS} libodometry)

#set(EXE_SIMPLE_NAME "demo_simple")
#add_executable(${EXE_SIMPLE_NAME} demo/demo_simple.cpp)
#target_link_libraries(${EXE_SIMPLE_NAME} ${DEMO_LIBS})

set(EXE_DEMO_NAME "demo_vo_homo")
add_executable(${EXE_DEMO_NAME} demo/demo_vo_homo.cpp)
target_link_libraries(${EXE_DEMO_NAME} ${DEMO_LIBS})
