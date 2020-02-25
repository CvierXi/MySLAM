# - LIB_CORE

add_subdirectory(../core Core)

aux_source_directory(. DIR_ODOMETRY)
add_library(libodometry ${DIR_ODOMETRY})
target_link_libraries(libodometry libcore)