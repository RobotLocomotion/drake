# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

# TODO(jwnimmer-tri) This BUILD file only compiles the in-tree solvers.
# External solvers are not yet supported.

cc_library(
    name = "constraint",
    srcs = ["constraint.cc"],
    hdrs = ["constraint.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:polynomial",
        "//drake/math:matrix_util",
    ],
)

cc_library(
    name = "mathematical_program",
    srcs = [
        "decision_variable.cc",
        "dreal_solver.h",
        "equality_constrained_qp_solver.cc",
        "equality_constrained_qp_solver.h",
        "gurobi_solver.h",
        "ipopt_solver.h",
        "linear_system_solver.cc",
        "linear_system_solver.h",
        "mathematical_program.cc",
        "moby_lcp_solver.cc",
        "moby_lcp_solver.h",
        "mosek_solver.h",
        "nlopt_solver.h",
        "no_dreal.cc",
        "no_gurobi.cc",
        "no_ipopt.cc",
        "no_mosek.cc",
        "no_nlopt.cc",
        "no_snopt.cc",
        "snopt_solver.h",
    ],
    hdrs = [
        "constraint.h",
        "decision_variable.h",
        "function.h",
        "mathematical_program.h",
        "solution_result.h",
    ],
    linkstatic = 1,
    deps = [
        ":constraint",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:number_traits",
        "//drake/common:polynomial",
        "//drake/math:matrix_util",
    ],
)

cc_library(
    name = "system_identification",
    srcs = ["system_identification.cc"],
    hdrs = ["system_identification.h"],
    linkstatic = 1,
    deps = [
        ":mathematical_program",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:polynomial",
    ],
)

# === test/ ===

cc_test(
    name = "constraint_test",
    size = "small",
    srcs = ["test/constraint_test.cc"],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:gradient",
        "@gtest//:main",
    ],
)

cc_test(
    name = "convex_optimization_test",
    size = "small",
    srcs = [
        "test/convex_optimization_test.cc",
        "test/mathematical_program_test_util.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "decision_variable_test",
    size = "small",
    srcs = ["test/decision_variable_test.cc"],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "mathematical_program_test",
    size = "small",
    srcs = [
        "test/mathematical_program_test.cc",
        "test/mathematical_program_test_util.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "mixed_integer_optimization_test",
    size = "small",
    srcs = [
        "test/mathematical_program_test_util.h",
        "test/mixed_integer_optimization_test.cc",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "moby_lcp_solver_test",
    size = "small",
    srcs = ["test/moby_lcp_solver_test.cc"],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_identification_test",
    size = "medium",
    srcs = ["test/system_identification_test.cc"],
    deps = [
        ":system_identification",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)
