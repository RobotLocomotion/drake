if(lcm_FOUND)

  add_library_with_exports(LIB_NAME drakeSchunkGripper SOURCE_FILES
    simulated_schunk_system.cc)
  target_link_libraries(drakeSchunkGripper
    drakeRigidBodyPlant)
  pods_install_libraries(drakeSchunkGripper)
  drake_install_headers(
          simulated_schunk_system.h)
  pods_install_pkg_config_file(drake-schunk-gripper
          LIBS -ldrakeSchunkGripper
          REQUIRES
          VERSION 0.0.1)
endif()

add_subdirectory(test)
