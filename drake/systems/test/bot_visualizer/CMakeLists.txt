if(lcm_FOUND)
  add_executable(bot_visualizer_test_one_sdf_box
      bot_visualizer_test_one_sdf_box.cc
      bot_visualizer_system_receiver.cc
      compare_lcm_messages.cc)
  target_link_libraries(bot_visualizer_test_one_sdf_box
      drakeBotVisualizerSystem
      drakeLCMSystem2
      lcm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bot_visualizer_test_one_sdf_box COMMAND
      bot_visualizer_test_one_sdf_box)

  add_executable(bot_visualizer_test_one_sdf_sphere
      bot_visualizer_test_one_sdf_sphere.cc
      bot_visualizer_system_receiver.cc
      compare_lcm_messages.cc)
  target_link_libraries(bot_visualizer_test_one_sdf_sphere
      drakeBotVisualizerSystem
      drakeLCMSystem2
      lcm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bot_visualizer_test_one_sdf_sphere COMMAND
      bot_visualizer_test_one_sdf_sphere)

  add_executable(bot_visualizer_test_one_sdf_cylinder
      bot_visualizer_test_one_sdf_cylinder.cc
      bot_visualizer_system_receiver.cc
      compare_lcm_messages.cc)
  target_link_libraries(bot_visualizer_test_one_sdf_cylinder
      drakeBotVisualizerSystem
      drakeLCMSystem2
      lcm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bot_visualizer_test_one_sdf_cylinder COMMAND
      bot_visualizer_test_one_sdf_cylinder)

  add_executable(bot_visualizer_test_one_sdf_capsule
      bot_visualizer_test_one_sdf_capsule.cc
      bot_visualizer_system_receiver.cc
      compare_lcm_messages.cc)
  target_link_libraries(bot_visualizer_test_one_sdf_capsule
      drakeBotVisualizerSystem
      drakeLCMSystem2
      lcm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bot_visualizer_test_one_sdf_capsule COMMAND
      bot_visualizer_test_one_sdf_capsule)
endif()
