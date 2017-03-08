if(Bullet_FOUND)
  drake_add_cc_test(model_test)
  target_link_libraries(model_test drakeCollision)

  drake_add_cc_test(collision_filter_group_test)
  target_link_libraries(collision_filter_group_test
      drakeCollision
      drakeMultibodyParsers
      drakeRBM
      )
endif()

if(ccd_FOUND AND fcl_FOUND AND octomap_FOUND)
  drake_add_cc_test(fcl_model_test)
  target_link_libraries(fcl_model_test drakeCollision)
endif()
