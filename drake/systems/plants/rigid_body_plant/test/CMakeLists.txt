find_package(Bullet)

if(BULLET_FOUND)
  add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
  target_link_libraries(rigid_body_plant_test
      drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)

  add_executable(viewer_draw_translator_1_test
      viewer_draw_translator_1_test.cc)
  target_link_libraries(viewer_draw_translator_1_test
      drake_rbp
      drakeLCMSystem2
      drakeRBSystem
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME viewer_draw_translator_1_test COMMAND
      viewer_draw_translator_1_test)

  add_executable(rigid_body_tree_visualizer_lcm_1_test
      rigid_body_tree_visualizer_lcm_1_test.cc)
  target_link_libraries(rigid_body_tree_visualizer_lcm_1_test
      drake_rbp
      drakeLCMSystem2
      drakeRBSystem
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_tree_visualizer_lcm_1_test COMMAND
      rigid_body_tree_visualizer_lcm_1_test)
endif()
