find_package(Bullet)

if(BULLET_FOUND)
  add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
  target_link_libraries(rigid_body_plant_test
      drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)

  add_executable(lcmt_viewer_draw_translator_test_1
      lcmt_viewer_draw_translator_test_1.cc)
  target_link_libraries(lcmt_viewer_draw_translator_test_1
      drake_rbp
      drakeLCMSystem2
      drakeRBSystem
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME lcmt_viewer_draw_translator_test_1 COMMAND
      lcmt_viewer_draw_translator_test_1)
endif()
