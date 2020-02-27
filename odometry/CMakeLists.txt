# - LIB_CORE

add_subdirectory(../core Core)

aux_source_directory(. DIR_ODOMETRY)
add_library(odometry ${DIR_ODOMETRY})
target_link_libraries(odometry core)