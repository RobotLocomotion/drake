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
endif()
