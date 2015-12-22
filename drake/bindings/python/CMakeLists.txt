function(add_python_test module_name)
	add_test(NAME test_python_${module_name}
		COMMAND ${PYTHON_EXECUTABLE} -m unittest ${module_name})
endfunction(add_python_test)

if (snopt_c_FOUND)
	add_python_test(pydrake.test.testRBMIK)
endif()
add_python_test(pydrake.test.testRBMForwardKin)
add_python_test(pydrake.test.testRBMCoM)
