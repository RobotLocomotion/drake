drake_add_cc_test(pose_aggregator_test)
target_link_libraries(pose_aggregator_test drakeRendering)

drake_add_cc_test(pose_vector_test)
target_link_libraries(pose_vector_test drakeRendering)

if(lcm_FOUND)
  drake_add_cc_test(drake_visualizer_client_test)
  target_link_libraries(drake_visualizer_client_test drakeRenderingLcm drakeLCMTypes)

  drake_add_cc_test(pose_stamped_t_pose_vector_translator_test)
  target_link_libraries(pose_stamped_t_pose_vector_translator_test
      drakeRendering
      drakeRenderingLcm)
endif()
