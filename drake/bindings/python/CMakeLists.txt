function(add_python_test module_name)
	add_test(NAME test_python_${module_name}
		COMMAND ${PYTHON_EXECUTABLE} -m unittest ${module_name}
		WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)
endfunction(add_python_test)

if (TARGET ikPython)
	add_python_test(dragon.test.testRBMIK)
endif()
add_python_test(dragon.test.testRBMForwardKin)
add_python_test(dragon.test.testRBMCoM)
