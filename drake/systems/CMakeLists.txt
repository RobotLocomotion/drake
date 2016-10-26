add_subdirectory(analysis)
add_subdirectory(framework)
add_subdirectory(lcm)

# ==== Below this line is MATLAB-specific code ====

if(Matlab_SIMULINK_FOUND)
  drake_add_mex(DCSFunction DCSFunction.cpp)
endif()

add_subdirectory(frames)

# ==== Below this line is C++ and MATLAB shared code ====

add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)
