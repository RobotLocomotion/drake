if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa world demos based on
  # System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaWorld SOURCE_FILES
      world_sim_tree_builder.cc
      world_sim_diagram_factory.cc)
  target_link_libraries(drakeKukaIiwaWorld
      drakeKukaIiwaArmCommon
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      gflags
      spruce)
  drake_install_libraries(drakeKukaIiwaWorld)
  drake_install_headers(
      world_sim_tree_builder.h
      world_sim_diagram_factory.h)
  drake_install_pkg_config_file(drake-kuka-iiwa-world
      TARGET drakeKukaIiwaWorld
      LIBS
        -ldrakeKukaIiwaWorld
        -lgflags
      REQUIRES
        drake-kuka-iiwa-arm-common
        drake-lcm-system2
        drake-multibody-parsers
        drake-rigid-body-plant
        drake-system-analysis
        drake-system-framework
        drake-system-primitives
        spruce)

  add_executable(iiwa_world_demo iiwa_world_demo.cc)
  target_link_libraries(iiwa_world_demo
      drakeKukaIiwaWorld)
  drake_add_test(
      NAME iiwa_world_demo COMMAND iiwa_world_demo -simulation_sec 0.1)
endif()
