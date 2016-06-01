if(bullet_FOUND AND GTEST_FOUND)
  add_executable(model_test model_test.cc)
  target_link_libraries(model_test drakeCollision ${GTEST_BOTH_LIBRARIES})
  add_test(NAME model_test COMMAND model_test)
endif()

add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions COMMAND testActiveCollisionOptions)
