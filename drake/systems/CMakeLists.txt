
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

add_library(drakeSystem DrakeSystem.cpp)

add_subdirectory(test)