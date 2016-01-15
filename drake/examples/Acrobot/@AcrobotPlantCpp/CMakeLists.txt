if (MATLAB_FOUND)
	add_mex(manipulatorDynamics manipulatorDynamics.cpp)
	target_link_libraries(manipulatorDynamics drakeMexUtil)
endif()
