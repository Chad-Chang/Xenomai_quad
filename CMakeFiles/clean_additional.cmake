# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles/RT_ECAT_MASTER_ELMO_GOLD_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/RT_ECAT_MASTER_ELMO_GOLD_autogen.dir/ParseCache.txt"
  "RT_ECAT_MASTER_ELMO_GOLD_autogen"
  )
endif()
