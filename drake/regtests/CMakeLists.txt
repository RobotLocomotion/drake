# This directory contains black-box regression tests that operate on the
# installed outputs of the Drake build.

if(MATLAB_FOUND)
  add_matlab_test(NAME addpath_spotless_test COMMAND "addpath_spotless;")
endif()
