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

add_subdirectory(util)

if(BUILD_TESTING)
  function(drake_add_matlab_check_dependency_test dependency)
    drake_add_matlab_test(NAME matlab/check_dependency_${dependency}
      REQUIRES ${dependency} COMMAND "checkDependency('${dependency}')"
      CHECK_DEPENDENCY_STRICT SIZE small)
  endfunction()

  drake_add_matlab_check_dependency_test(lcm)

  # Unknown dependency.
  drake_add_matlab_test(NAME matlab/check_dependency_unknown
    COMMAND "checkDependency('unknown')" CHECK_DEPENDENCY_STRICT)
  set_tests_properties(matlab/check_dependency_unknown PROPERTIES WILL_FAIL ON)

  function(drake_add_matlab_check_dependency_test_no dependency)
    if(NOT ${dependency}_FOUND)
      drake_add_matlab_test(NAME matlab/check_dependency_no_${dependency}
        COMMAND "checkDependency('${dependency}')" CHECK_DEPENDENCY_STRICT
        SIZE small)
      set_tests_properties(matlab/check_dependency_no_${dependency}
        PROPERTIES WILL_FAIL ON)
    endif()
  endfunction()

  # TODO(jamiesnape): This would still pass if MATLAB fails for a reason
  # other than a missing dependency.
  drake_add_matlab_check_dependency_test_no(lcm)
endif()
