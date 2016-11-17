# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:common", "@gtest//:main"]

cc_test(
    name = "autodiff_overloads_test",
    srcs = ["autodiff_overloads_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:autodiff"])

cc_test(
    name = "drake_assert_test",
    srcs = ["drake_assert_test.cc"],
    size = "small", deps = DEPS)

cc_test(
    name = "drake_assert_test_compile",
    srcs = ["drake_assert_test_compile.cc"],
    size = "small", deps = DEPS)

cc_test(
    name = "drake_deprecated_test",
    srcs = ["drake_deprecated_test.cc"],
    size = "small", deps = DEPS,
    # Remove spurious warnings from the default build output.
    copts = ["-Wno-deprecated-declarations"])

cc_test(
    name = "drake_throw_test",
    srcs = ["drake_throw_test.cc"],
    size = "small", deps = DEPS)

cc_test(
    name = "eigen_matrix_compare_test",
    srcs = ["eigen_matrix_compare_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:eigen_matrix_compare"])

cc_test(
    name = "eigen_stl_types_test",
    srcs = ["eigen_stl_types_test.cc"],
    size = "small", deps = DEPS)

cc_test(
    name = "functional_form_test",
    srcs = ["functional_form_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:functional_form"])

cc_test(
    name = "nice_type_name_test",
    srcs = ["nice_type_name_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:nice_type_name"])

cc_test(
    name = "polynomial_test",
    srcs = ["polynomial_test.cc", "random_polynomial_matrix.h"],
    size = "small", deps = DEPS + ["//drake/common:polynomial", "//drake/common:eigen_matrix_compare"])

cc_test(
    name = "sorted_vectors_have_intersection_test",
    srcs = ["sorted_vectors_have_intersection_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:sorted_vectors_have_intersection"])

cc_test(
    name = "symbolic_environment_test",
    srcs = ["symbolic_environment_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:symbolic"])

cc_test(
    name = "symbolic_expression_test",
    srcs = ["symbolic_expression_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:symbolic"])

cc_test(
    name = "symbolic_formula_test",
    srcs = ["symbolic_formula_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:symbolic"])

cc_test(
    name = "symbolic_variable_test",
    srcs = ["symbolic_variable_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:symbolic"])

cc_test(
    name = "symbolic_variables_test",
    srcs = ["symbolic_variables_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:symbolic"])

cc_test(
    name = "text_logging_test",
    srcs = ["text_logging_test.cc"],
    size = "small", deps = DEPS)

cc_test(
    name = "trig_poly_test",
    srcs = ["trig_poly_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:polynomial"])

# TODO(jwnimmer-tri) These tests are currently missing...
# - drake_assert_test in fancy variants
# - drake_assert_test_compile in fancy variants
# - text_logging_test in fancy variants
# - drake_deprecated_test in fancy variants
# - cpplint_wrapper_test.py
