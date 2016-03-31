if (LCM_FOUND AND GTEST_FOUND) # LCM is only needed because it's using Pendulum.h for examples...
  add_executable(vector_test vector_test.cc)
  target_link_libraries(vector_test ${GTEST_BOTH_LIBRARIES})
  add_dependencies(vector_test drake_lcmtypes lcmtype_agg_hpp)
  add_test(NAME vector_test COMMAND vector_test)
endif (LCM_FOUND AND GTEST_FOUND)