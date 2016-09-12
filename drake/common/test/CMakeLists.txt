# Build tests for Drake common utilities.

add_executable(functional_form_test functional_form_test.cc)
target_link_libraries(functional_form_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME functional_form_test COMMAND functional_form_test)

add_executable(nice_type_name_test nice_type_name_test.cc)
target_link_libraries(nice_type_name_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME nice_type_name_test COMMAND nice_type_name_test)

add_executable(sorted_vectors_have_intersection_test
        sorted_vectors_have_intersection_test.cc)
target_link_libraries(sorted_vectors_have_intersection_test
        drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME sorted_vectors_have_intersection_test
         COMMAND sorted_vectors_have_intersection_test)

add_executable(trig_poly_test trig_poly_test.cc)
target_link_libraries(trig_poly_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME trig_poly_test COMMAND trig_poly_test)

add_executable(polynomial_test polynomial_test.cc)
target_link_libraries(polynomial_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME polynomial_test COMMAND polynomial_test)

add_executable(eigen_matrix_compare_test eigen_matrix_compare_test.cc)
target_link_libraries(eigen_matrix_compare_test ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME eigen_matrix_compare_test COMMAND eigen_matrix_compare_test)

add_executable(eigen_stl_types_test eigen_stl_types_test.cc)
target_link_libraries(eigen_stl_types_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME eigen_stl_types_test COMMAND eigen_stl_types_test)

add_executable(drake_throw_test drake_throw_test.cc)
target_link_libraries(drake_throw_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME drake_throw_test COMMAND drake_throw_test)

# Adds "drake_assert.h" unit testing.
add_executable(drake_assert_test_default drake_assert_test.cc)
target_link_libraries(drake_assert_test_default drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME drake_assert_test_default COMMAND drake_assert_test_default)
# Same, but with assertions forced enabled.
add_executable(drake_assert_test_enabled drake_assert_test.cc)
target_compile_definitions(drake_assert_test_enabled PRIVATE DRAKE_ENABLE_ASSERTS)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(drake_assert_test_enabled PRIVATE -UDRAKE_DISABLE_ASSERTS)
endif()
target_link_libraries(drake_assert_test_enabled drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME drake_assert_test_enabled COMMAND drake_assert_test_enabled)
# Same, but with assertions forced disabled.
add_executable(drake_assert_test_disabled drake_assert_test.cc)
target_compile_definitions(drake_assert_test_disabled PRIVATE DRAKE_DISABLE_ASSERTS)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(drake_assert_test_disabled PRIVATE -UDRAKE_ENABLE_ASSERTS)
endif()
target_link_libraries(drake_assert_test_disabled drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME drake_assert_test_disabled COMMAND drake_assert_test_disabled)

# "text_logging.h" unit testing:  If spdlog is available, test that:
add_executable(text_logging_test_default
               ../text_logging.cc text_logging_test.cc)
target_link_libraries(text_logging_test_default
                      ${GTEST_BOTH_LIBRARIES} gflags threads)
drake_add_test(NAME text_logging_test_default COMMAND text_logging_test_default)
# Force spdlog to unavailable to test that the ifndef(HAVE_SPDLOG) case works.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
    OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_executable(text_logging_test_no_spdlog
                 ../text_logging.cc text_logging_test.cc)
  target_compile_options(text_logging_test_no_spdlog
    PRIVATE -UHAVE_SPDLOG)
  target_link_libraries(text_logging_test_no_spdlog
                        ${GTEST_BOTH_LIBRARIES} gflags threads)
  drake_add_test(NAME text_logging_test_no_spdlog COMMAND text_logging_test_no_spdlog)
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

# Adds "drake_deprecated.h" unit testing.
add_executable(drake_deprecated_test drake_deprecated_test.cc)
target_link_libraries(drake_deprecated_test drakeCommon ${GTEST_BOTH_LIBRARIES})
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Remove spurious warnings from the default build output.
  target_compile_options(
    drake_deprecated_test
    PRIVATE -Wno-deprecated-declarations)
endif()
drake_add_test(NAME drake_deprecated_test COMMAND drake_deprecated_test)

# This block proves that DRAKE_DEPRECATED does cause deprecation warnings,
# by promoting that warning to an error and looking for compile failure,
# similar to the assert "nocompile" test above.  We cannot run this test
# on Windows, because drake_deprecated.h forces C4996 to be a warning; as
# the code exists now, we cannot promote to an error via the command line.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_executable(
    drake_deprecated_test_nocompile
    drake_deprecated_test.cc)
  target_link_libraries(
    drake_deprecated_test_nocompile
    drakeCommon ${GTEST_BOTH_LIBRARIES})
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
endif()

# Rules for cpplint_wrapper.py.
if(PYTHONINTERP_FOUND AND WITH_GOOGLE_STYLEGUIDE)
  drake_add_test(NAME cpplint_wrapper_test COMMAND
    ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/cpplint_wrapper_test.py)
endif()
