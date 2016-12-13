drake_add_matlab_test(NAME examples/ZMP/CartTable.run OPTIONAL bullet gurobi COMMAND CartTable.run)
drake_add_matlab_test(NAME examples/ZMP/CartTable2D.run OPTIONAL bullet gurobi COMMAND CartTable2D.run)

add_library_with_exports(LIB_NAME drakeZMPTestUtil SOURCE_FILES zmp_test_util.cc)
target_link_libraries(drakeZMPTestUtil drakeZMPPlanner)
drake_install_libraries(drakeZMPTestUtil)
drake_install_headers(zmp_test_util.h)
drake_install_pkg_config_file(drake-zmp-test_util
  TARGET drakeZMPTestUtil
  LIBS -ldrakeZMPTestUtil)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

if (lcm_FOUND)
  add_executable(zmp_example
      zmp_example.cc)
  target_link_libraries(zmp_example
      drakeZMPTestUtil
      drakeLcm)
endif()
