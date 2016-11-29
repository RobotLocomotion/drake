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

  add_executable(schunk_simulation schunk_simulation.cc)
  target_link_libraries(schunk_simulation
    drakeSchunkGripper
    drakeSystemAnalysis
    drakeSystemControllers
    drakeLCMSystem2)
  drake_add_test(
      NAME schunk_simulation COMMAND schunk_simulation -simulation_sec 0.1)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
