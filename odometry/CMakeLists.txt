# - LIB_ODOMETRY

add_subdirectory(../core Core)
add_subdirectory(../visualize Vis)

aux_source_directory(. DIR_ODOMETRY)
add_library(odometry ${DIR_ODOMETRY})
target_link_libraries(odometry core vis)