if(Bullet_FOUND)
  drake_add_cc_test(model_test)
  target_link_libraries(model_test drakeCollision)
endif()
