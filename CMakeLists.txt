cmake_minimum_required(VERSION 3.15)
project(MySLAM_dev)

set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

include(cmake_modules/FindOpenCV.cmake)

add_library(libconfig_parse config_parse/config_parser.cpp)
set(DEMO_LIBS ${DEMO_LIBS} ${THIRD_LIBS} libconfig_parse)

set(EXE_SIMPLE_NAME "demo_simple")
add_executable(${EXE_SIMPLE_NAME} demo/demo_simple.cpp)
target_link_libraries(${EXE_SIMPLE_NAME} ${DEMO_LIBS})

set(EXE_DEMO_NAME "demo_vo")
add_executable(${EXE_DEMO_NAME} demo/demo_vo.cpp)
target_link_libraries(${EXE_DEMO_NAME} ${DEMO_LIBS})
