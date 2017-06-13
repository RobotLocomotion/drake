drake_add_matlab_test(NAME examples/PR2/drawPR2 OPTIONAL bullet COMMAND drawPR2)
drake_add_matlab_test(NAME examples/PR2/runSaggitalPassive COMMAND runSaggitalPassive)  # FIXME: see #3313
# Attempt to only add the test when not in the OPEN_SOURCE CI configuration
# FIXME: see #3444
if(WITH_IRIS OR WITH_SEDUMI)
  drake_add_matlab_test(NAME examples/PR2/runPassive COMMAND runPassive SIZE large)
endif()

if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa world demos based on
  # System 2.0.
  add_library_with_exports(LIB_NAME drakePR2 SOURCE_FILES
          simulation_utils/world_sim_tree_builder.cc simulation_utils/sim_diagram_builder.cc)
  target_link_libraries(drakePR2
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
  drake_install_libraries(drakePR2)
  drake_install_headers(
          simulation_utils/world_sim_tree_builder.h simulation_utils/sim_diagram_builder.h)
  drake_install_pkg_config_file(drake-pr2
      TARGET drakePR2
      LIBS
        -ldrakePR2
        -lgflags
      REQUIRES
        drake-pr2
        drake-lcm-system2
        drake-multibody-parsers
        drake-rigid-body-plant
        drake-sim-diagram-builder
        drake-system-analysis
        drake-system-framework
        drake-system-primitives
        oracular-state-estimator
        spruce)

  add_executable(pr2_simulation pr2_simulation.cc)
  target_link_libraries(pr2_simulation
      drakePR2
      drakeSimDiagramBuilder
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers)
endif()
