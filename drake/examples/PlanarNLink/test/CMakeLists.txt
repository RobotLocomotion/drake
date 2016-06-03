add_matlab_test(NAME examples/PlanarNLink/test/bodyKinTest COMMAND bodyKinTest)

if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/PlanarNLink/test/polyIK COMMAND polyIK PROPERTIES TIMEOUT 3000 REQUIRES spotless)
endif()
