drake_add_matlab_test(NAME examples/ZMP/CartTable.run OPTIONAL bullet gurobi COMMAND CartTable.run)
drake_add_matlab_test(NAME examples/ZMP/CartTable2D.run OPTIONAL bullet gurobi COMMAND CartTable2D.run)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum.runPassive COMMAND LinearInvertedPendulum.runPassive)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum.runLQR COMMAND LinearInvertedPendulum.runLQR)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum.ZMPtrackingDemo COMMAND LinearInvertedPendulum.ZMPtrackingDemo)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum2D.runPassive COMMAND LinearInvertedPendulum2D.runPassive)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum2D.runLQR COMMAND LinearInvertedPendulum2D.runLQR)
drake_add_matlab_test(NAME examples/ZMP/LinearInvertedPendulum2D.ZMPtrackingDemo COMMAND LinearInvertedPendulum2D.ZMPtrackingDemo)

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
