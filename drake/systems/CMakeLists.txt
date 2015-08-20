
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()


add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

include_directories(${PROJECT_SOURCE_DIR}/systems/frames)
add_library(drakeSystem DrakeSystem.cpp)

add_subdirectory(test)