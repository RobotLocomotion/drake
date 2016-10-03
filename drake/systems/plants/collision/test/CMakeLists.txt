find_package(Bullet)

if(BULLET_FOUND)
  drake_add_cc_test(model_test)
  target_link_libraries(model_test drakeCollision)
endif()

drake_add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions OPTIONAL bullet COMMAND testActiveCollisionOptions)
