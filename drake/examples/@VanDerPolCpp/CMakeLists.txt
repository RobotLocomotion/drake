if (MATLAB_FOUND)
	add_mex(dynamicsRHS dynamicsRHS.cpp)
	target_link_libraries(dynamicsRHS drakeMexUtil drakePolynomial)
endif()
