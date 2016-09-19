find_package(Bullet)

if(BULLET_FOUND)
  add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
  target_link_libraries(rigid_body_plant_test
      drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)

  add_executable(viewer_draw_translator_test
      viewer_draw_translator_test.cc)
  target_link_libraries(viewer_draw_translator_test
      drake_rbp
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME viewer_draw_translator_test COMMAND
      viewer_draw_translator_test)

  add_executable(rigid_body_tree_visualizer_lcm_test
      rigid_body_tree_visualizer_lcm_test.cc)
  target_link_libraries(rigid_body_tree_visualizer_lcm_test
      drake_rbp
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_tree_visualizer_lcm_test COMMAND
      rigid_body_tree_visualizer_lcm_test)
endif()
