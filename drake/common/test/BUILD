# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/common:common",
    "@gtest//:main",
]

cc_library(
    name = "random_polynomial_matrix",
    testonly = 1,
    hdrs = ["random_polynomial_matrix.h"],
    linkstatic = 1,
    deps = ["//drake/common:polynomial"],
)

cc_test(
    name = "autodiff_overloads_test",
    size = "small",
    srcs = ["autodiff_overloads_test.cc"],
    deps = DEPS + [
        "//drake/common:autodiff",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_test(
    name = "double_overloads_test",
    size = "small",
    srcs = ["double_overloads_test.cc"],
    deps = DEPS + [
        "//drake/common:double",
        "//drake/common:cond",
    ],
)

cc_test(
    name = "drake_assert_test",
    size = "small",
    srcs = ["drake_assert_test.cc"],
    deps = DEPS,
)

cc_test(
    name = "drake_assert_test_compile",
    size = "small",
    srcs = ["drake_assert_test_compile.cc"],
    deps = DEPS,
)

cc_test(
    name = "drake_deprecated_test",
    size = "small",
    srcs = ["drake_deprecated_test.cc"],
    # Remove spurious warnings from the default build output.
    copts = ["-Wno-deprecated-declarations"],
    deps = DEPS,
)

cc_test(
    name = "drake_throw_test",
    size = "small",
    srcs = ["drake_throw_test.cc"],
    deps = DEPS,
)

cc_test(
    name = "eigen_matrix_compare_test",
    size = "small",
    srcs = ["eigen_matrix_compare_test.cc"],
    deps = DEPS + ["//drake/common:eigen_matrix_compare"],
)

cc_test(
    name = "eigen_stl_types_test",
    size = "small",
    srcs = ["eigen_stl_types_test.cc"],
    deps = DEPS,
)

cc_test(
    name = "functional_form_test",
    size = "small",
    srcs = ["functional_form_test.cc"],
    deps = DEPS + ["//drake/common:functional_form"],
)

cc_test(
    name = "is_approx_equal_abstol_test",
    srcs = ["is_approx_equal_abstol_test.cc"],
    size = "small", deps = DEPS + ["//drake/common:is_approx_equal_abstol"])

cc_test(
    name = "nice_type_name_test",
    size = "small",
    srcs = ["nice_type_name_test.cc"],
    deps = DEPS + ["//drake/common:nice_type_name"],
)

cc_test(
    name = "polynomial_test",
    size = "small",
    srcs = ["polynomial_test.cc"],
    deps = DEPS + [
        ":random_polynomial_matrix",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_test(
    name = "sorted_vectors_have_intersection_test",
    size = "small",
    srcs = ["sorted_vectors_have_intersection_test.cc"],
    deps = DEPS + ["//drake/common:sorted_vectors_have_intersection"],
)

cc_test(
    name = "symbolic_environment_test",
    size = "small",
    srcs = ["symbolic_environment_test.cc"],
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_test(
    name = "symbolic_expression_test",
    size = "small",
    srcs = ["symbolic_expression_test.cc"],
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_test(
    name = "symbolic_formula_test",
    size = "small",
    srcs = ["symbolic_formula_test.cc"],
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_test(
    name = "symbolic_variable_test",
    size = "small",
    srcs = ["symbolic_variable_test.cc"],
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_test(
    name = "symbolic_variables_test",
    size = "small",
    srcs = ["symbolic_variables_test.cc"],
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_test(
    name = "text_logging_test",
    size = "small",
    srcs = ["text_logging_test.cc"],
    deps = DEPS,
)

cc_test(
    name = "trig_poly_test",
    size = "small",
    srcs = ["trig_poly_test.cc"],
    deps = DEPS + ["//drake/common:polynomial"],
)

# TODO(jwnimmer-tri) These tests are currently missing...
# - drake_assert_test in fancy variants
# - drake_assert_test_compile in fancy variants
# - text_logging_test in fancy variants
# - drake_deprecated_test in fancy variants
# - cpplint_wrapper_test.py
