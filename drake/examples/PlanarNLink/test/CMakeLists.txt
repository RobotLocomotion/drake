add_matlab_test(NAME examples/PlanarNLink/test/bodyKinTest COMMAND bodyKinTest)

if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/PlanarNLink/test/polyIK COMMAND polyIK REQUIRES gloptipoly3 spotless PROPERTIES TIMEOUT 3000)
endif()
