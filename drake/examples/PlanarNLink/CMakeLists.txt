# drake_add_matlab_test(NAME examples/PlanarNLink/runLQR COMMAND runLQR)  # FIXME: see #3313
drake_add_matlab_test(NAME examples/PlanarNLink/runPassive OPTIONAL bullet COMMAND runPassive)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
