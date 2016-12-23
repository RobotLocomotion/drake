if(Bullet_FOUND)
  drake_add_cc_test(model_test)
  target_link_libraries(model_test drakeCollision)
endif()

if(ccd_FOUND AND fcl_FOUND AND octomap_FOUND)
  drake_add_cc_test(fcl_model_test)
  target_link_libraries(fcl_model_test drakeCollision)
endif()
