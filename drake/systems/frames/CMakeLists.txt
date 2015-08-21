
include_directories(${PROJECT_SOURCE_DIR}/systems)

set(coordinate_frame_files CoordinateFrame.cpp)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  list(APPEND coordinate_frame_files LCMCoordinateFrame.cpp)
endif()
add_library(drakeCoordinateFrame ${coordinate_frame_files})
if (LCM_FOUND)
  add_dependencies(drakeCoordinateFrame drake_lcmtypes)
  pods_use_pkg_config_packages(drakeCoordinateFrame lcm)
endif()

add_subdirectory(@CoordinateFrame)




