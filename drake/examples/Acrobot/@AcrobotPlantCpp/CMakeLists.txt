if(Matlab_FOUND)
  drake_add_mex(manipulatorDynamics manipulatorDynamics.cpp)
  target_link_libraries(manipulatorDynamics drakeMexUtil)
endif()
