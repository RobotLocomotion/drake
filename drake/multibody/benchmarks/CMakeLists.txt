add_subdirectory(cylinder_torque_free_analytical_solution)
add_subdirectory(acrobot)

add_library_with_exports(LIB_NAME drakeMultibodyBenchmarks SOURCE_FILES
  acrobot/acrobot.cc)
target_link_libraries(drakeMultibodyBenchmarks
  drakeCommon)
drake_install_libraries(drakeMultibodyBenchmarks)
drake_install_pkg_config_file(drake-multibody-benchmarks
  TARGET drakeMultibodyBenchmarks
  LIBS -drakeMultibodyBenchmarks
  REQUIRES
  drake-common
  eigen3)

