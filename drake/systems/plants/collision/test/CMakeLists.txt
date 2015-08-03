include_directories ( .. )

if (bullet_FOUND)

if (NOT (APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
  # it's silly, but i haven't gotten this to link properly with clang
  # on the mac yet... - Russ

  add_executable( model_test ModelTest.cpp)
  target_link_libraries(model_test drakeCollision)
  add_test(NAME model_test_test COMMAND model_test)
endif()

endif()
