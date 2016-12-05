configure_file(addpath_drake.m.in addpath_drake.m @ONLY)
configure_file(rmpath_drake.m.in rmpath_drake.m @ONLY)

configure_file(get_drake_binary_dir.m.in get_drake_binary_dir.m @ONLY)
configure_file(get_drake_install_dir.m.in get_drake_install_dir.m @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/addpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/rmpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_binary_dir.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_install_dir.m"
  DESTINATION matlab)

add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)

drake_add_matlab_test(NAME addpath_spotless_test COMMAND "addpath_spotless;")
