drake_find_package(Bullet MODULE)

if(Bullet_FOUND)
  drake_add_cc_test(rigid_body_plant_test)
  target_link_libraries(rigid_body_plant_test drakeRigidBodyPlant)

  drake_add_cc_test(rigid_body_plant_system1_test)
  target_link_libraries(rigid_body_plant_system1_test
    drakeRigidBodyPlant drakeRBSystem)

  if (lcm_FOUND)
    drake_add_cc_test(viewer_draw_translator_test)
    target_link_libraries(viewer_draw_translator_test drakeRigidBodyPlant)

    drake_add_cc_test(drake_visualizer_test)
    target_link_libraries(drake_visualizer_test
      drakeRigidBodyPlant)
  endif()
endif()
