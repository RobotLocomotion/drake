if(Matlab_SIMULINK_FOUND)
  drake_add_mex(DCSFunction DCSFunction.cpp)
endif()

add_subdirectory(frames)

add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
