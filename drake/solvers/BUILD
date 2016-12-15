# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

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
    name = "decision_variable",
    srcs = ["decision_variable.cc"],
    hdrs = ["decision_variable.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:number_traits",
    ],
)

cc_library(
    name = "function",
    srcs = [],
    hdrs = ["function.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "solution_result",
    srcs = [],
    hdrs = ["solution_result.h"],
    linkstatic = 1,
    deps = [],
)

# Supported solvers:
# - equality_constrained_qp_solver
# - linear_system_solver
# - moby_lcp_solver
# - Gurobi (on Ubuntu)

# Unsupported solvers:
# - dReal
# - IPOPT
# - NLOPT
# - SNOPT
# - MOSEK

# TODO(jwnimmer-tri, david-german-tri): Support more solvers.
cc_library(
    name = "mathematical_program",
    srcs = [
        "dreal_solver.h",
        "equality_constrained_qp_solver.cc",
        "equality_constrained_qp_solver.h",
        "ipopt_solver.h",
        "linear_system_solver.cc",
        "linear_system_solver.h",
        "mathematical_program.cc",
        "moby_lcp_solver.cc",
        "moby_lcp_solver.h",
        "mosek_solver.h",
        "nlopt_solver.h",
        "no_dreal.cc",
        "no_ipopt.cc",
        "no_mosek.cc",
        "no_nlopt.cc",
        "no_snopt.cc",
        "snopt_solver.h",
    ],
    hdrs = [
        "constraint.h",
        "mathematical_program.h",
    ],
    linkstatic = 1,
    deps = [
        ":constraint",
        ":decision_variable",
        ":function",
        ":gurobi_solver",
        ":solution_result",
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

# The mathematical_program_api target exists so that solver targets can access
# the mathematical_program.h header, and the :mathematical_program target can
# depend on solver targets, without creating a circular dependency.
#
# TODO(david-german-tri): MathematicalProgram should be refactored so that
# implementations of MathematicalProgramSolverInterface are not co-dependent
# with MathematicalProgram itself, and then this target should be deleted.
cc_library(
    name = "mathematical_program_api",
    srcs = [],
    hdrs = ["mathematical_program.h"],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":constraint",
        ":decision_variable",
        ":function",
        ":solution_result",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:number_traits",
        "//drake/common:polynomial",
    ],
)

# External Solvers.

# If --config gurobi is used, this target builds object code that satisfies
# MathematicalProgramSolverInterface by calling Gurobi. Otherwise, this target
# builds dummy object code that fails at runtime.
cc_library(
    name = "gurobi_solver",
    srcs = select({
        "//tools:with_gurobi": ["gurobi_solver.cc"],
        "//conditions:default": ["no_gurobi.cc"],
    }),
    hdrs = ["gurobi_solver.h"],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = select({
        "//tools:with_gurobi": [
            ":mathematical_program_api",
            "//drake/math:eigen_sparse_triplet",
            "@gurobi//:lib",
        ],
        "//conditions:default": [
            ":mathematical_program_api",
        ],
    }),
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
    # This test uses Gurobi if present, but does not require it.
    tags = [
        "exclusive",
        "local",
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
    # This test uses Gurobi if present, but does not require it.
    tags = [
        "exclusive",
        "local",
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
