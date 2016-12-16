# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

# TODO(jwnimmer-tri) This BUILD file only compiles the in-tree solvers.
# External solvers are not yet supported.

cc_component(
    name = "constraint",
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

cc_component(
    name = "system_identification",
    deps = [
        ":mathematical_program",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:polynomial",
    ],
)

# === test/ ===

cc_unit_test(
    name = "constraint_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:gradient",
    ],
)

cc_unit_test(
    name = "convex_optimization_test",
    srcs = [
        "test/convex_optimization_test.cc",
        "test/mathematical_program_test_util.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "decision_variable_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "mathematical_program_test",
    srcs = [
        "test/mathematical_program_test.cc",
        "test/mathematical_program_test_util.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "mixed_integer_optimization_test",
    srcs = [
        "test/mathematical_program_test_util.h",
        "test/mixed_integer_optimization_test.cc",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "moby_lcp_solver_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "system_identification_test",
    deps = [
        ":system_identification",
        "//drake/common:eigen_matrix_compare",
    ],
)
