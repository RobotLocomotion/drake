if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaWorld SOURCE_FILES
      iiwa_world_simulator.cc)
  target_link_libraries(drakeKukaIiwaWorld
      drakeKukaIiwaArmCommon)
  pods_install_libraries(drakeKukaIiwaWorld)
  drake_install_headers(
      iiwa_world_simulator.h)
  pods_install_pkg_config_file(drake-kuka-iiwa-world
      LIBS -ldrakeKukaIiwaWorld
      REQUIRES
      VERSION 0.0.1)

  add_executable(iiwa_world_demo iiwa_world_demo.cc)
  target_link_libraries(iiwa_world_demo
      drakeKukaIiwaWorld
      drakeSystemAnalysis
      drakeLCMSystem2
      gflags)
#  drake_add_test(
#      NAME iiwa_world_demo
#      COMMAND iiwa_world_demo --duration 0.5)
endif()