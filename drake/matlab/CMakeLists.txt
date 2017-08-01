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

if(lcm_FOUND)
  # Needs to be in cache to keep MATLAB happy. The target lcm-java is imported
  # by find_package(lcm).
  get_target_property(_lcm_jar_file lcm-java JAR_FILE)
  get_filename_component(LCM_JAR_FILE "${_lcm_jar_file}" REALPATH CACHE)
endif()

add_subdirectory(solvers/test)
add_subdirectory(util)


