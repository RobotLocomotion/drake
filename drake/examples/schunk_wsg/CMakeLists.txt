if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSchunkWsg SOURCE_FILES
    gen/schunk_wsg_trajectory_generator_state_vector.cc
    schunk_wsg_lcm.cc
    simulated_schunk_wsg_system.cc)
  target_link_libraries(drakeSchunkWsg
    drakeLCMSystem2
    drakeMultibodyParsers
    drakeRigidBodyPlant)
  drake_install_libraries(drakeSchunkWsg)
  drake_install_headers(
    # N.B. The gen/*.h headers are installed by gen/CMakeLists.txt.
    schunk_wsg_lcm.cc
    simulated_schunk_wsg_system.h)
  drake_install_pkg_config_file(drake-schunk-wsg
    TARGET drakeSchunkWsg
    LIBS -ldrakeSchunkWsg
    REQUIRES
      drake-lcm-system
      drake-lcmtypes-cpp
      drake-multibody-parsers
      drake-rigid-body-plant)

  add_executable(schunk_wsg_simulation schunk_wsg_simulation.cc)
  target_link_libraries(schunk_wsg_simulation
    drakeSchunkWsg
    drakeSystemAnalysis
    drakeSystemControllers
    drakeLCMSystem2)
  drake_add_test(
      NAME schunk_wsg_simulation COMMAND schunk_wsg_simulation -simulation_sec 0.1)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
