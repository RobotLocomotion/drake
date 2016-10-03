find_package(Bullet)

if(BULLET_FOUND)
  drake_add_cc_test(rigid_body_plant_test)
  target_link_libraries(rigid_body_plant_test drakeRigidBodyPlant drakeRBSystem)

  if (lcm_FOUND)
    drake_add_cc_test(viewer_draw_translator_test)
    target_link_libraries(viewer_draw_translator_test drakeRigidBodyPlant)

    drake_add_cc_test(rigid_body_tree_lcm_publisher_test)
    target_link_libraries(rigid_body_tree_lcm_publisher_test
      drakeRigidBodyPlant)
  endif()
endif()
