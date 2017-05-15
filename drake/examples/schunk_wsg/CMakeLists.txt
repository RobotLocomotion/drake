add_library_with_exports(LIB_NAME drakeSimulatedSchunkWsg SOURCE_FILES
  simulated_schunk_wsg_system.cc)
target_link_libraries(drakeSimulatedSchunkWsg
  drakeMultibodyParsers
  drakeRigidBodyPlant)
drake_install_libraries(drakeSimulatedSchunkWsg)
drake_install_headers(
  simulated_schunk_wsg_system.h)
drake_install_pkg_config_file(drake-simulated-schunk-wsg
    TARGET drakeSimulatedSchunkWsg
    LIBS -ldrakeSimulatedSchunkWsg
    REQUIRES
      drake-multibody-parsers
      drake-rigid-body-plant)

if(lcm_FOUND)
  add_executable(schunk_wsg_simulation schunk_wsg_simulation.cc)
  target_link_libraries(schunk_wsg_simulation
    drakeLcmSystem
    drakeRigidBodyPlant
    drakeSchunkWsg
    drakeSimulatedSchunkWsg
    drakeSystemAnalysis
    drakeSystemControllers)
  drake_add_test(
      NAME schunk_wsg_simulation COMMAND schunk_wsg_simulation --simulation_sec 0.1)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
