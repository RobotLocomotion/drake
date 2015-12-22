function(add_python_test module_name)
	add_test(NAME test_python_${module_name}
		COMMAND ${PYTHON_EXECUTABLE} -m unittest ${module_name})
endfunction(add_python_test)

add_python_test(pydrake.test.testRBMIK)
add_python_test(pydrake.test.testRBMForwardKin)
add_python_test(pydrake.test.testRBMCoM)
