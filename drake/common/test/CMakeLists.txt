# Build tests for Drake common utilities.

drake_add_cc_test(functional_form_test)
target_link_libraries(functional_form_test drakeCommon)

drake_add_cc_test(nice_type_name_test)
target_link_libraries(nice_type_name_test drakeCommon)

drake_add_cc_test(symbolic_variable_test)
target_link_libraries(symbolic_variable_test drakeCommon)

drake_add_cc_test(symbolic_variables_test)
target_link_libraries(symbolic_variables_test drakeCommon)

drake_add_cc_test(symbolic_environment_test)
target_link_libraries(symbolic_environment_test drakeCommon)

drake_add_cc_test(symbolic_expression_test)
target_link_libraries(symbolic_expression_test drakeCommon)

drake_add_cc_test(symbolic_formula_test)
target_link_libraries(symbolic_formula_test drakeCommon)

drake_add_cc_test(sorted_vectors_have_intersection_test)
target_link_libraries(sorted_vectors_have_intersection_test drakeCommon)

drake_add_cc_test(trig_poly_test)
target_link_libraries(trig_poly_test drakeCommon)

drake_add_cc_test(polynomial_test)
target_link_libraries(polynomial_test drakeCommon)

drake_add_cc_test(eigen_matrix_compare_test)
drake_add_cc_test(eigen_stl_types_test)

drake_add_cc_test(drake_throw_test)
target_link_libraries(drake_throw_test drakeCommon)

# Adds "drake_assert.h" unit testing.
add_executable(drake_assert_test_default drake_assert_test.cc)
target_link_libraries(drake_assert_test_default drakeCommon GTest::GTest GTest::Main)
drake_add_test(NAME drake_assert_test_default COMMAND drake_assert_test_default)
# Same, but with assertions forced enabled.
add_executable(drake_assert_test_enabled drake_assert_test.cc)
target_compile_definitions(drake_assert_test_enabled PRIVATE DRAKE_ENABLE_ASSERTS)
target_compile_options(drake_assert_test_enabled PRIVATE -UDRAKE_DISABLE_ASSERTS)
target_link_libraries(drake_assert_test_enabled drakeCommon GTest::GTest GTest::Main)
drake_add_test(NAME drake_assert_test_enabled COMMAND drake_assert_test_enabled)
# Same, but with assertions forced disabled.
add_executable(drake_assert_test_disabled drake_assert_test.cc)
target_compile_definitions(drake_assert_test_disabled PRIVATE DRAKE_DISABLE_ASSERTS)
target_compile_options(drake_assert_test_disabled PRIVATE -UDRAKE_ENABLE_ASSERTS)
target_link_libraries(drake_assert_test_disabled drakeCommon GTest::GTest GTest::Main)
drake_add_test(NAME drake_assert_test_disabled COMMAND drake_assert_test_disabled)

# "text_logging.h" unit testing:  If spdlog is available, test that:
add_executable(text_logging_test_default ../text_logging.cc text_logging_test.cc)
target_link_libraries(text_logging_test_default GTest::GTest GTest::Main gflags)
drake_add_test(NAME text_logging_test_default COMMAND text_logging_test_default)
# Force spdlog to unavailable to test that the ifndef(HAVE_SPDLOG) case works.
add_executable(text_logging_test_no_spdlog ../text_logging.cc text_logging_test.cc)
target_compile_options(text_logging_test_no_spdlog PRIVATE -UHAVE_SPDLOG)
target_link_libraries(text_logging_test_no_spdlog GTest::GTest GTest::Main gflags)
drake_add_test(NAME text_logging_test_no_spdlog COMMAND text_logging_test_no_spdlog)

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
drake_add_test(NAME
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
drake_add_test(NAME
  drake_assert_test_nocompile
  COMMAND ${CMAKE_COMMAND} --build . --target
  drake_assert_test_nocompile --config $<CONFIGURATION>
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(
  drake_assert_test_nocompile
  PROPERTIES WILL_FAIL TRUE)
# Same, but with assertions forced enabled.
add_executable(
  drake_assert_test_nocompile_enabled
  drake_assert_test_compile.cc)
target_compile_definitions(
  drake_assert_test_nocompile_enabled PRIVATE DRAKE_ASSERT_TEST_COMPILE_ERROR)
set_target_properties(
  drake_assert_test_nocompile_enabled
  PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
drake_add_test(NAME
  drake_assert_test_nocompile_enabled
  COMMAND ${CMAKE_COMMAND} --build . --target
  drake_assert_test_nocompile_enabled --config $<CONFIGURATION>
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(
  drake_assert_test_nocompile_enabled
  PROPERTIES WILL_FAIL TRUE)
target_compile_definitions(
  drake_assert_test_nocompile_enabled PRIVATE DRAKE_ENABLE_ASSERTS)
target_compile_options(
  drake_assert_test_nocompile_enabled
  PRIVATE -UDRAKE_DISABLE_ASSERTS)
# Same, but with assertions forced disabled.
add_executable(
  drake_assert_test_nocompile_disabled
  drake_assert_test_compile.cc)
target_compile_definitions(
  drake_assert_test_nocompile_disabled PRIVATE DRAKE_ASSERT_TEST_COMPILE_ERROR)
set_target_properties(
  drake_assert_test_nocompile_disabled
  PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
drake_add_test(NAME
  drake_assert_test_nocompile_disabled
  COMMAND ${CMAKE_COMMAND} --build . --target
  drake_assert_test_nocompile_disabled --config $<CONFIGURATION>
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(
  drake_assert_test_nocompile_disabled
  PROPERTIES WILL_FAIL TRUE)
target_compile_definitions(
  drake_assert_test_nocompile_disabled
  PRIVATE DRAKE_DISABLE_ASSERTS)
target_compile_options(
  drake_assert_test_nocompile_disabled
  PRIVATE -UDRAKE_ENABLE_ASSERTS)

# Adds "drake_deprecated.h" unit testing.
drake_add_cc_test(drake_deprecated_test)
target_link_libraries(drake_deprecated_test drakeCommon)
# Remove spurious warnings from the default build output.
target_compile_options(drake_deprecated_test PRIVATE -Wno-deprecated-declarations)

# This block proves that DRAKE_DEPRECATED does cause deprecation warnings,
# by promoting that warning to an error and looking for compile failure,
# similar to the assert "nocompile" test above.
add_executable(
  drake_deprecated_test_nocompile
  drake_deprecated_test.cc)
target_link_libraries(
  drake_deprecated_test_nocompile
  drakeCommon GTest::GTest GTest::Main)
set_target_properties(
  drake_deprecated_test_nocompile
  PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
target_compile_options(
  drake_deprecated_test_nocompile
  PRIVATE -Werror=deprecated-declarations)
drake_add_test(NAME
  drake_deprecated_test_nocompile
  COMMAND ${CMAKE_COMMAND} --build . --target
  drake_deprecated_test_nocompile --config $<CONFIGURATION>
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(
  drake_deprecated_test_nocompile
  PROPERTIES WILL_FAIL TRUE)

# Rules for cpplint_wrapper.py.
if(WITH_GOOGLE_STYLEGUIDE)
  drake_add_test(NAME cpplint_wrapper_test COMMAND
    ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/cpplint_wrapper_test.py)
endif()
