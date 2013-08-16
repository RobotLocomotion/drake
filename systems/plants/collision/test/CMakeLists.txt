include_directories ( .. )

find_package( Boost COMPONENTS unit_test_framework test_exec_monitor)

if (Boost_FOUND)
  add_executable( body_test BodyTest.cpp)
  target_link_libraries(body_test drakeCollision ${Boost_LIBRARIES})
  add_test(body_test_test body_test)
endif()

