configure_file(addpath_drake.m.in "${PROJECT_BINARY_DIR}/addpath_drake.m" @ONLY)

if(BUILD_TESTING)
  add_subdirectory(solvers/test)
endif()

add_subdirectory(util)
