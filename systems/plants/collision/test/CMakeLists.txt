include_directories ( .. )

find_package( Boost COMPONENTS unit_test_framework test_exec_monitor)

if (Boost_FOUND)

if (NOT (APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
  # it's silly, but i haven't gotten this to link properly with clang
  # on the mac yet... - Russ

  add_executable( body_test BodyTest.cpp)
  target_link_libraries(body_test drakeCollision ${Boost_LIBRARIES})
  add_test(body_test_test body_test)
endif()

endif()

