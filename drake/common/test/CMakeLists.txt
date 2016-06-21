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

# The entire block of CMake build rules and CTests exists to confirm that the
# disarmed DRAKE_ASSERT still yields compilation errors.
#
# Here is the long-form explanation of what is happening:
#
# The drake_assert_test_compile.cc file is compiled twice; once that compiles
# successfully (by cmake), and is run (by ctest), and returns 0; and once that
# fails to compile (under ctest).  This is to confirm that the compile failure
# is due to the DRAKE_ASSERT statement only, and not some unrelated problem.
#
# The DRAKE_ASSERT_TEST_COMPILE_ERROR macro is defined when we want to induce
# a compile failure; thus, only the second test defines that macro.
#
# Both the first and second version are compiled and tested by default using
# only the default CMake and CTest invocations.  Drake developers do not need
# to invoke any special commands to run these tests.
#
# - This first version of the test should trivially pass.
add_executable(
  drake_assert_test_compile 
  drake_assert_test_compile.cc)
add_test(NAME 
  drake_assert_test_compile COMMAND
  drake_assert_test_compile)
# - This second version of the test should yield a compile error, as follows:
#  - add_executable: Declare a compilation rule to create an executable.
#  - target_compile_definitions: Enable code that generates a compile error.
#  - set_target_properties: Do not compile the executable as part of the build.
#  - add_test: Add a test that asks cmake to compile the executable.
#  - set_test_properties: The test passes if and only if the exitcode was 1.
add_executable(
  drake_assert_test_nocompile 
  drake_assert_test_compile.cc)
target_compile_definitions(
  drake_assert_test_nocompile PRIVATE DRAKE_ASSERT_TEST_COMPILE_ERROR)
set_target_properties(
  drake_assert_test_nocompile
  PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
add_test(NAME 
  drake_assert_test_nocompile
  COMMAND ${CMAKE_COMMAND} --build . --target 
  drake_assert_test_nocompile --config $<CONFIGURATION> 
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(
  drake_assert_test_nocompile
  PROPERTIES WILL_FAIL TRUE)
