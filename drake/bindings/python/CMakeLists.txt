set(DRAKE_INSTALL_PYTHON_DIR
  "${DRAKE_LIBRARY_DIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages")

install(DIRECTORY pydrake DESTINATION ${DRAKE_INSTALL_PYTHON_DIR})

if(BUILD_TESTING)
  function(drake_add_python_test module_name)
    drake_add_test(NAME test_python_${module_name}
      COMMAND "${PYTHON_EXECUTABLE}" -m unittest ${module_name})
    set_tests_properties(test_python_${module_name} PROPERTIES ENVIRONMENT "PYTHONPATH=$PYTHONPATH:${DRAKE_INSTALL_PYTHON_DIR}")
  endfunction()

  drake_add_python_test(pydrake.test.testCommon)
  drake_add_python_test(pydrake.test.test_urdf_parser)
  drake_add_python_test(pydrake.test.testRBTTransformPoints)
  drake_add_python_test(pydrake.test.testRBTCoM)
  drake_add_python_test(pydrake.test.testForwardDiff)
  drake_add_python_test(pydrake.test.testSymbolic)
endif()
