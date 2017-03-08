if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa world demos based on
  # System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaWorld SOURCE_FILES
      world_sim_tree_builder.cc
      world_sim_diagram_factory.cc
      iiwa_wsg_diagram_factory.cc)
  target_link_libraries(drakeKukaIiwaWorld
      drakeKukaIiwaArmCommon
      drakeLcmSystem
      drakeLCMUtil
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSimDiagramBuilder
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      drakeSystemControllers
      oracularStateEstimator
      gflags
      spruce)
  drake_install_libraries(drakeKukaIiwaWorld)
  drake_install_headers(
      world_sim_tree_builder.h
      world_sim_diagram_factory.h
      iiwa_wsg_diagram_factory.h)
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
        drake-sim-diagram-builder
        drake-system-analysis
        drake-system-framework
        drake-system-primitives
        oracular-state-estimator
        spruce)

  add_executable(iiwa_world_demo iiwa_world_demo.cc)
  target_link_libraries(iiwa_world_demo
      drakeKukaIiwaWorld)
  drake_add_test(
      NAME iiwa_world_demo COMMAND iiwa_world_demo --simulation_sec 0.1)

  add_executable(controlled_iiwa_world_demo controlled_iiwa_world_demo.cc)
  target_link_libraries(controlled_iiwa_world_demo
      drakeKukaIiwaWorld)
  if(snopt_FOUND)
    drake_add_test(
        NAME controlled_iiwa_world_demo
        COMMAND controlled_iiwa_world_demo --simulation_sec 0.1)
  endif()

  add_executable(multi_arm_with_gripper_demo multi_arm_with_gripper_demo.cc)
  target_link_libraries(multi_arm_with_gripper_demo
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      drakeSimDiagramBuilder
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers)
endif()
