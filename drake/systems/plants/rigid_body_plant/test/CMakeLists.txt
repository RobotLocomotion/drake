if(bullet_FOUND)
  add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
  target_link_libraries(rigid_body_plant_test
      drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)

  add_executable(translator_between_vector_base_and_lcmt_viewer_draw_test
      translator_between_vector_base_and_lcmt_viewer_draw_test.cc)
  target_link_libraries(translator_between_vector_base_and_lcmt_viewer_draw_test
      drake_rbp
      drakeLCMSystem2
      drakeRBSystem
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME translator_between_vector_base_and_lcmt_viewer_draw_test COMMAND
      translator_between_vector_base_and_lcmt_viewer_draw_test)
endif()
