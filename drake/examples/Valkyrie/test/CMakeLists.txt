
if (0) # currently disabled because they take too long to run
  add_matlab_test(NAME examples/Valkyrie/test/runValkyrieFastWalking COMMAND runValkyrieFastWalking)
  add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning COMMAND runValkyrieTurning PROPERTIES  TIMEOUT  1000)
  add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning270 COMMAND runValkyrieTurning270 PROPERTIES  TIMEOUT  1000)
endif()
