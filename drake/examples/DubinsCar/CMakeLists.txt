# drake_add_matlab_test(NAME examples/DubinsCar/runFunnel COMMAND runFunnel)  # FIXME: see #3313

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
