# - LIB_CORE

aux_source_directory(./config_parse DIR_CONFIG_PARSE)
set(DIR_CORE ${DIR_CORE} ${DIR_CONFIG_PARSE})

aux_source_directory(./util DIR_UTIL)
set(DIR_CORE ${DIR_CORE} ${DIR_UTIL})

aux_source_directory(./dataset_parse DIR_DATASET_PARSE)
set(DIR_CORE ${DIR_CORE} ${DIR_DATASET_PARSE})

add_library(libcore ${DIR_CORE})