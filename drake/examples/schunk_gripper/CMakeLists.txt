if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSchunkGripper SOURCE_FILES
    simulated_schunk_system.cc)
  target_link_libraries(drakeSchunkGripper drakeRigidBodyPlant)
  drake_install_libraries(drakeSchunkGripper)
  drake_install_headers(simulated_schunk_system.h)
  drake_install_pkg_config_file(drake-schunk-gripper
    TARGET drakeSchunkGripper
    LIBS -ldrakeSchunkGripper
    REQUIRES drake-rigid-body-plant)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
