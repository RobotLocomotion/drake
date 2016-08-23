if(lcm_FOUND)
  add_executable(bot_visualizer_test_one_urdf_box
      bot_visualizer_test_one_urdf_box.cc
      bot_visualizer_system_receiver.cc)
  target_link_libraries(bot_visualizer_test_one_urdf_box
      drakeBotVisualizerSystem
      drakeLCMSystem2
      lcm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bot_visualizer_test_one_urdf_box COMMAND
      bot_visualizer_test_one_urdf_box)
endif()
