if(bullet_FOUND)
  add_executable(model_test model_test.cc)
  target_link_libraries(model_test drakeCollision ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME model_test COMMAND model_test)
endif()

drake_add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions OPTIONAL bullet COMMAND testActiveCollisionOptions)
