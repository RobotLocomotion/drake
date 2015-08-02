
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)
