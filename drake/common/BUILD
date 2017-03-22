# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

# A library of things that EVERYONE should want and MUST EAT.
# Be appropriately hesitant when adding new things here.
drake_cc_library(
    name = "common",
    srcs = [
        "drake_assert.cc",
        "drake_throw.cc",
        "text_logging.cc",
    ],
    hdrs = [
        "constants.h",
        "drake_assert.h",
        "drake_compat.h",
        "drake_copyable.h",
        "drake_deprecated.h",
        "drake_optional.h",
        "drake_throw.h",
        "eigen_stl_types.h",
        "eigen_types.h",
        "never_destroyed.h",
        "text_logging.h",
    ],
    deps = [
        "@eigen//:eigen",
        "@spdlog//:spdlog",
    ],
)

# Specific traits and operators that are relevant to all ScalarTypes.
drake_cc_library(
    name = "cond",
    srcs = ["cond.cc"],
    hdrs = ["cond.h"],
    deps = [
        ":double",
    ],
)

drake_cc_library(
    name = "dummy_value",
    hdrs = ["dummy_value.h"],
)

drake_cc_library(
    name = "number_traits",
    hdrs = ["number_traits.h"],
)

# Drake's specific ScalarType-providing libraries.
drake_cc_library(
    name = "double",
    srcs = ["double_overloads.cc"],
    hdrs = ["double_overloads.h"],
)

drake_cc_library(
    name = "autodiff",
    hdrs = [
        "autodiff_overloads.h",
        "eigen_autodiff_types.h",
    ],
    deps = [
        ":common",
        ":cond",
        ":dummy_value",
    ],
)

drake_cc_library(
    name = "extract_double",
    hdrs = ["extract_double.h"],
    deps = [
        ":common",
        ":nice_type_name",
        ":number_traits",
    ],
)

drake_cc_library(
    name = "functional_form",
    srcs = ["functional_form.cc"],
    hdrs = ["functional_form.h"],
    deps = [
        ":common",
        ":dummy_value",
    ],
)

drake_cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = [
        "polynomial.h",
        "trig_poly.h",
    ],
    deps = [
        ":autodiff",
        ":common",
    ],
)

drake_cc_library(
    name = "symbolic",
    srcs = [
        "symbolic_environment.cc",
        "symbolic_expression.cc",
        "symbolic_expression_cell.cc",
        "symbolic_expression_cell.h",
        "symbolic_expression_visitor.cc",
        "symbolic_formula.cc",
        "symbolic_formula_cell.cc",
        "symbolic_formula_cell.h",
        "symbolic_variable.cc",
        "symbolic_variables.cc",
    ],
    hdrs = [
        "hash.h",
        "symbolic_environment.h",
        "symbolic_expression.h",
        "symbolic_expression_visitor.h",
        "symbolic_formula.h",
        "symbolic_variable.h",
        "symbolic_variables.h",
    ],
    deps = [
        ":common",
        ":cond",
        ":dummy_value",
        ":number_traits",
        ":polynomial",
    ],
)

drake_cc_library(
    name = "symbolic_test_util",
    testonly = 1,
    hdrs = [
        "test/symbolic_test_util.h",
    ],
    deps = [
        ":symbolic",
    ],
)

drake_cc_library(
    name = "type_safe_index",
    srcs = [],
    hdrs = ["type_safe_index.h"],
    deps = [
        ":common",
        ":nice_type_name",
    ],
)

# Miscellaneous utilities.
drake_cc_library(
    name = "drake_path",
    # TODO(david-german-tri): Set testonly = 1 once the non-test uses of
    # drake_path are removed.
    # testonly = 1,
    srcs = ["drake_path.cc"],
    hdrs = ["drake_path.h"],
    deps = [
        ":common",
    ],
)

# TODO(jwnimmer-tri) GetDrakePath has a long and storied history (#1471, #2174).
# It serves multiple purposes (unit tests loading their models; installed demos
# loading their models; etc.) but doesn't really do any of them well.  The rule
# here in Bazel is intended as a temporary shim, in order to support Bazel and
# CMake build systems concurrently.  We should endeavor to remove this rule (in
# lieu of properly-declared data dependencies) once we are unchained from CMake.
genrule(
    name = "drake_path_genrule",
    srcs = ["drake_path.cc.in"],
    outs = ["drake_path.cc"],
    # TODO(jwnimmer-tri) This should be an abspath, not relpath.
    cmd = "sed 's#@PROJECT_SOURCE_DIR@#drake#g' $< > $@",
)

# TODO(jwnimmer-tri) Move this header file into test/ subdirectory (and this
# stanza below the "test/" comment marker).
drake_cc_library(
    name = "eigen_matrix_compare",
    testonly = 1,
    hdrs = ["eigen_matrix_compare.h"],
    deps = [
        ":common",
    ],
)

drake_cc_library(
    name = "is_approx_equal_abstol",
    hdrs = ["is_approx_equal_abstol.h"],
    deps = [
        ":common",
    ],
)

drake_cc_library(
    name = "monomial",
    srcs = ["monomial.cc"],
    hdrs = ["monomial.h"],
    deps = [
        ":common",
        ":symbolic",
    ],
)

drake_cc_library(
    name = "nice_type_name",
    srcs = ["nice_type_name.cc"],
    hdrs = ["nice_type_name.h"],
    deps = [
        ":common",
    ],
)

drake_cc_library(
    name = "reinit_after_move",
    hdrs = ["reinit_after_move.h"],
)

drake_cc_library(
    name = "sorted_vectors_have_intersection",
    hdrs = ["sorted_vectors_have_intersection.h"],
    deps = [
        ":common",
    ],
)

drake_cc_library(
    name = "text_logging_gflags",
    hdrs = ["text_logging_gflags.h"],
    # TODO(jwnimmer-tri) Ideally, gflags BUILD would do this for us.  Figure
    # out what's going on.  Definitely don't let "-pthread" get copy-pasta'd
    # throughout our code.
    linkopts = select({
        "//tools:linux": ["-pthread"],
        "//conditions:default": [],
    }),
    deps = [
        ":common",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_library(
    name = "random_polynomial_matrix",
    testonly = 1,
    hdrs = ["test/random_polynomial_matrix.h"],
    deps = [
        ":polynomial",
    ],
)

drake_cc_library(
    name = "measure_execution",
    testonly = 1,
    hdrs = ["test/measure_execution.h"],
)

drake_cc_googletest(
    name = "autodiff_overloads_test",
    deps = [
        ":autodiff",
        ":common",
        ":eigen_matrix_compare",
        ":extract_double",
    ],
)

drake_cc_googletest(
    name = "reinit_after_move_test",
    deps = [
        ":reinit_after_move",
    ],
)

drake_cc_googletest(
    name = "cond_test",
    deps = [
        ":cond",
    ],
)

drake_cc_googletest(
    name = "decompose_polynomial_test",
    deps = [
        ":common",
        ":monomial",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "double_overloads_test",
    deps = [
        ":common",
        ":cond",
        ":double",
    ],
)

drake_cc_googletest(
    name = "drake_assert_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "drake_assert_test_compile",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "drake_deprecated_test",
    # Remove spurious warnings from the default build output.
    copts = ["-Wno-deprecated-declarations"],
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "drake_optional_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "drake_throw_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "dummy_value_test",
    deps = [
        ":dummy_value",
    ],
)

drake_cc_googletest(
    name = "eigen_matrix_compare_test",
    deps = [
        ":common",
        ":eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "eigen_stl_types_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "extract_double_test",
    deps = [
        ":common",
        ":extract_double",
    ],
)

drake_cc_googletest(
    name = "functional_form_test",
    deps = [
        ":common",
        ":functional_form",
    ],
)

drake_cc_googletest(
    name = "is_approx_equal_abstol_test",
    deps = [
        ":common",
        ":is_approx_equal_abstol",
    ],
)

drake_cc_googletest(
    name = "monomial_test",
    deps = [
        ":common",
        ":monomial",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "never_destroyed_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "nice_type_name_test",
    deps = [
        ":common",
        ":nice_type_name",
    ],
)

drake_cc_googletest(
    name = "polynomial_test",
    deps = [
        ":common",
        ":eigen_matrix_compare",
        ":random_polynomial_matrix",
    ],
)

drake_cc_googletest(
    name = "sorted_vectors_have_intersection_test",
    deps = [
        ":common",
        ":sorted_vectors_have_intersection",
    ],
)

drake_cc_googletest(
    name = "symbolic_environment_test",
    deps = [
        ":common",
        ":symbolic",
    ],
)

drake_cc_googletest(
    name = "symbolic_expansion_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_expression_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_expression_differentiation_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_expression_jacobian_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_expression_matrix_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_mixing_scalar_types_test",
    deps = [
        ":common",
        ":symbolic",
    ],
)

drake_cc_googletest(
    name = "symbolic_substitution_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_variable_overloading_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_formula_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_variable_test",
    deps = [
        ":common",
        ":symbolic",
        ":symbolic_test_util",
    ],
)

drake_cc_googletest(
    name = "symbolic_variables_test",
    deps = [
        ":common",
        ":symbolic",
    ],
)

drake_cc_googletest(
    name = "text_logging_test",
    deps = [
        ":common",
    ],
)

drake_cc_googletest(
    name = "trig_poly_test",
    deps = [
        ":common",
        ":polynomial",
    ],
)

drake_cc_googletest(
    name = "type_safe_index_test",
    deps = [":type_safe_index"],
)

# TODO(jwnimmer-tri) These tests are currently missing...
# - drake_assert_test in fancy variants
# - drake_assert_test_compile in fancy variants
# - text_logging_test in fancy variants
# - drake_deprecated_test in fancy variants
# - cpplint_wrapper_test.py

cpplint()
