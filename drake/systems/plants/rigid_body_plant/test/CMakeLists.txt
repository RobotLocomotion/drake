find_package(Bullet)

if(BULLET_FOUND)
  add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
  target_link_libraries(rigid_body_plant_test
      drakeRigidBodyPlant drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)

  if (lcm_FOUND)
    add_executable(viewer_draw_translator_test
        viewer_draw_translator_test.cc)
    target_link_libraries(viewer_draw_translator_test
        drakeRigidBodyPlant
        ${GTEST_BOTH_LIBRARIES})
    add_test(NAME viewer_draw_translator_test COMMAND
        viewer_draw_translator_test)

    add_executable(rigid_body_tree_lcm_publisher_test
        rigid_body_tree_lcm_publisher_test.cc)
    target_link_libraries(rigid_body_tree_lcm_publisher_test
        drakeRigidBodyPlant
        ${GTEST_BOTH_LIBRARIES})
    add_test(NAME rigid_body_tree_lcm_publisher_test COMMAND
        rigid_body_tree_lcm_publisher_test)
  endif()
endif()
