include_directories ( .. )

find_package( Boost COMPONENTS unit_test_framework test_exec_monitor)

if (Boost_FOUND AND bullet_FOUND)

if (NOT (APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
  # it's silly, but i haven't gotten this to link properly with clang
  # on the mac yet... - Russ

  add_executable( body_test BodyTest.cpp)
  add_executable( model_test ModelTest.cpp)
  add_executable( primitive_distance_test primitiveTest.cpp)
  target_link_libraries(body_test drakeCollision ${Boost_LIBRARIES})
  target_link_libraries(model_test drakeCollision ${Boost_LIBRARIES})
  target_link_libraries(primitive_distance_test drakeCollision ${Boost_LIBRARIES})
  add_test(NAME body_test_test COMMAND body_test)
  add_test(NAME model_test_test COMMAND model_test)
  add_test(NAME primitive_distance_test_test COMMAND body_test)
endif()

endif()
