configure_file(addpath_drake.m.in addpath_drake.m @ONLY)
configure_file(rmpath_drake.m.in rmpath_drake.m @ONLY)

configure_file(get_drake_binary_dir.m.in get_drake_binary_dir.m @ONLY)
configure_file(get_drake_install_dir.m.in get_drake_install_dir.m @ONLY)

configure_file(drake_get_base_path.m.in drake_get_base_path.m @ONLY)
configure_file(drake_get_bin_path.m.in drake_get_bin_path.m @ONLY)
configure_file(drake_get_lib_path.m.in drake_get_lib_path.m @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/addpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/rmpath_drake.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_binary_dir.m"
  "${CMAKE_CURRENT_BINARY_DIR}/get_drake_install_dir.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_base_path.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_bin_path.m"
  "${CMAKE_CURRENT_BINARY_DIR}/drake_get_lib_path.m"
  DESTINATION matlab)

add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)

if(BUILD_TESTING)
  drake_add_matlab_test(NAME "matlab/RigidBodyManipulatorMemoryTest"
    OPTIONAL bullet
    COMMAND "r = RigidBodyManipulator('Acrobot.urdf'), megaclear"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Acrobot SIZE large)

  drake_add_matlab_test(NAME addpath_spotless_test REQUIRES spotless
    COMMAND addpath_spotless)
endif()
