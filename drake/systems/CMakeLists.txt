
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

set(coordinate_frame_files)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  list(APPEND coordinate_frame_files frames/LCMCoordinateFrame.cpp)
endif()
add_library(drakeSystem DrakeSystem.cpp LinearSystem.cpp ${coordinate_frame_files})
target_link_libraries(drakeSystem drakeUtil)
if (LCM_FOUND)
  add_dependencies(drakeSystem drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(drakeSystem lcm)
endif()

add_subdirectory(test)