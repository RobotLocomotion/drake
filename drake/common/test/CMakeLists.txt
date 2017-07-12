# Build tests for Drake common utilities.

drake_add_cc_test(autodiff_overloads_test)
target_link_libraries(autodiff_overloads_test drakeCommon)

drake_add_cc_test(copyable_unique_ptr_test)
target_link_libraries(copyable_unique_ptr_test drakeCommon)

drake_add_cc_test(double_overloads_test)
target_link_libraries(double_overloads_test drakeCommon)

drake_add_cc_test(drake_optional_test)
target_link_libraries(double_overloads_test drakeCommon)

drake_add_cc_test(drake_throw_test)
target_link_libraries(drake_throw_test drakeCommon)

drake_add_cc_test(dummy_value_test)
target_link_libraries(dummy_value_test drakeCommon)

drake_add_cc_test(eigen_autodiff_types_test)
target_link_libraries(eigen_autodiff_types_test drakeCommon)

drake_add_cc_test(eigen_matrix_compare_test)
target_link_libraries(eigen_matrix_compare_test drakeCommon)

drake_add_cc_test(eigen_stl_types_test)
target_link_libraries(eigen_stl_types_test Eigen3::Eigen)

drake_add_cc_test(extract_double_test)
target_link_libraries(extract_double_test drakeCommon)

# TODO(jwnimmer-tri) Drake CI's ROS configuration uses an out-of-source build,
# which this unit test does not yet support:
#   drake_add_cc_test(find_resource_test)
add_executable(find_resource_test find_resource_test.cc)
target_link_libraries(find_resource_test drakeCommon GTest::GTest GTest::Main)

drake_add_cc_test(functional_form_test)
target_link_libraries(functional_form_test drakeCommon)

drake_add_cc_test(is_approx_equal_abstol_test)
target_link_libraries(is_approx_equal_abstol_test drakeCommon)

drake_add_cc_test(is_cloneable_test)
target_link_libraries(is_cloneable_test drakeCommon)

drake_add_cc_test(never_destroyed_test)
target_link_libraries(never_destroyed_test drakeCommon)

drake_add_cc_test(monomial_test)
target_link_libraries(monomial_test drakeCommon)

drake_add_cc_test(is_dynamic_castable_test)
target_link_libraries(is_dynamic_castable_test drakeCommon)

drake_add_cc_test(is_memcpy_movable_test)
target_link_libraries(is_memcpy_movable_test drakeCommon)

drake_add_cc_test(nice_type_name_test)
target_link_libraries(nice_type_name_test drakeCommon)

drake_add_cc_test(symbolic_environment_test)
target_link_libraries(symbolic_environment_test drakeCommon)

drake_add_cc_test(symbolic_expansion_test)
target_link_libraries(symbolic_expansion_test drakeCommon)

drake_add_cc_test(symbolic_expression_test)
target_link_libraries(symbolic_expression_test drakeCommon)

drake_add_cc_test(symbolic_expression_differentiation_test)
target_link_libraries(symbolic_expression_differentiation_test drakeCommon)

drake_add_cc_test(symbolic_expression_jacobian_test)
target_link_libraries(symbolic_expression_jacobian_test drakeCommon)

drake_add_cc_test(symbolic_expression_array_test)
target_link_libraries(symbolic_expression_array_test drakeCommon)

drake_add_cc_test(symbolic_expression_matrix_test)
target_link_libraries(symbolic_expression_matrix_test drakeCommon)

drake_add_cc_test(symbolic_substitution_test)
target_link_libraries(symbolic_substitution_test drakeCommon)

drake_add_cc_test(symbolic_mixing_scalar_types_test)
target_link_libraries(symbolic_mixing_scalar_types_test drakeCommon)

drake_add_cc_test(symbolic_formula_test)
target_link_libraries(symbolic_formula_test drakeCommon)

drake_add_cc_test(symbolic_formula_visitor_test)
target_link_libraries(symbolic_formula_visitor_test drakeCommon)

drake_add_cc_test(sorted_vectors_have_intersection_test)
target_link_libraries(sorted_vectors_have_intersection_test drakeCommon)

drake_add_cc_test(trig_poly_test)
target_link_libraries(trig_poly_test drakeCommon)

drake_add_cc_test(polynomial_test)
target_link_libraries(polynomial_test drakeCommon)

drake_add_cc_test(symbolic_variable_test)
target_link_libraries(symbolic_variable_test drakeCommon)

drake_add_cc_test(symbolic_variable_overloading_test)
target_link_libraries(symbolic_variable_overloading_test drakeCommon)

drake_add_cc_test(symbolic_polynomial_test)
target_link_libraries(symbolic_polynomial_test drakeCommon)

drake_add_cc_test(symbolic_variables_test)
target_link_libraries(symbolic_variables_test drakeCommon)

drake_add_cc_test(type_safe_index_test type_safe_index_test.cc)
target_link_libraries(type_safe_index_test drakeCommon)

drake_add_cc_test(drake_assert_test)
target_link_libraries(drake_assert_test drakeCommon)

drake_add_cc_test(drake_assert_test_compile)
target_link_libraries(drake_assert_test_compile drakeCommon)

drake_add_cc_test(scoped_singleton_test scoped_singleton_test.cc)
target_link_libraries(scoped_singleton_test drakeCommon)

# "text_logging.h" unit testing:  If spdlog is available, test that:
add_executable(text_logging_test_default ../text_logging.cc text_logging_test.cc)
target_link_libraries(text_logging_test_default GTest::GTest GTest::Main gflags)
drake_add_test(NAME text_logging_test_default COMMAND text_logging_test_default)
# Force spdlog to unavailable to test that the ifndef(HAVE_SPDLOG) case works.
add_executable(text_logging_test_no_spdlog ../text_logging.cc text_logging_test.cc)
target_compile_options(text_logging_test_no_spdlog PRIVATE -UHAVE_SPDLOG)
target_link_libraries(text_logging_test_no_spdlog GTest::GTest GTest::Main gflags)
drake_add_test(NAME text_logging_test_no_spdlog COMMAND text_logging_test_no_spdlog)

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
