if (fcl_FOUND)
  set(Boost_DEBUG TRUE)
  find_package(Boost REQUIRED system)
  add_executable( fcl_model_test FCLModelTest.cpp)
  target_link_libraries(fcl_model_test drakeCollision ${Boost_LIBRARIES})
endif()

if (bullet_FOUND AND GTEST_FOUND)
  add_executable(model_test model_test.cc)
  target_link_libraries(model_test drakeCollision ${GTEST_BOTH_LIBRARIES})
  add_test(NAME model_test COMMAND model_test)
endif()

add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions COMMAND testActiveCollisionOptions)
