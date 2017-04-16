if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSimpleTreeVisualizer SOURCE
      simple_tree_visualizer.cc)
  target_link_libraries(
      drakeLcmSystem
      drakeLCMUtil
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemFramework
  )
  drake_install_libraries(drakeSimpleTreeVisualizer)
  drake_install_header(
      simple_tree_visualizer.h)
  drake_install_pkg_config_file(drake-simple-tree-visualizer
      TARGET drakeSimpleTreeVisualizer
      LIBS
        -ldrakeSimpleTreeVisualizer
      REQUIRES
        drake-lcm-system2
        drake-multibody-parsers
        drake-rigid-body-plant
        drake-system-framework)

  add_executable(simple_tree_visualizer_demo simple_tree_visualizer_demo.cc)
  target_link_libraries(simple_tree_visualizer_demo
      simple_tree_visualizer)
  drake_add_test(
      NAME simple_tree_visualizer_demo COMMAND simple_tree_visualizer_demo
      --num_configurations 1)

endif()
