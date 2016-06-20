# Build tests for Drake common utilities.
#
if(GTEST_FOUND)
  add_executable(nice_type_name_test nice_type_name_test.cc)
  target_link_libraries(nice_type_name_test drakeCommon ${GTEST_BOTH_LIBRARIES})
  add_test(NAME nice_type_name_test COMMAND nice_type_name_test)

  add_executable(drake_assert_test_default drake_assert_test.cc)
  target_link_libraries(drake_assert_test_default drakeCommon ${GTEST_BOTH_LIBRARIES})
  add_test(NAME drake_assert_test_default COMMAND drake_assert_test_default)
  # Same, but with assertions forced enabled.
  add_executable(drake_assert_test_enabled drake_assert_test.cc)
  target_compile_definitions(drake_assert_test_enabled PRIVATE DRAKE_ENABLE_ASSERTS)
  target_link_libraries(drake_assert_test_enabled drakeCommon ${GTEST_BOTH_LIBRARIES})
  add_test(NAME drake_assert_test_enabled COMMAND drake_assert_test_enabled)
  # Same, but with assertions forced disabled.
  add_executable(drake_assert_test_disabled drake_assert_test.cc)
  target_compile_definitions(drake_assert_test_disabled PRIVATE DRAKE_DISABLE_ASSERTS)
  target_link_libraries(drake_assert_test_disabled drakeCommon ${GTEST_BOTH_LIBRARIES})
  add_test(NAME drake_assert_test_disabled COMMAND drake_assert_test_disabled)
endif()
