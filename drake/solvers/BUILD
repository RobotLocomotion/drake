# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

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
        "mathematical_program_solver_interface.h",
    ],
    linkstatic = 1,
    deps = [
        ":constraint",
        ":decision_variable",
        ":function",
        ":gurobi_solver",
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
# TODO(hongkai-dai): MathematicalProgram should be refactored so that
# implementations of MathematicalProgramSolverInterface are not co-dependent
# with MathematicalProgram itself, and then this target should be deleted.
cc_library(
    name = "mathematical_program_api",
    srcs = [],
    hdrs = [
        "mathematical_program.h",
        "mathematical_program_solver_interface.h",
    ],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":constraint",
        ":decision_variable",
        ":function",
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

cc_googletest(
    name = "constraint_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:gradient",
    ],
)

# TODO(hongkai-dai): Separate this test into a test that uses Gurobi, and a
# test that doesn't.
cc_googletest(
    name = "convex_optimization_test",
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
    ],
)

cc_googletest(
    name = "decision_variable_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
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

# TODO(hongkai-dai): Separate this test into a test that uses Gurobi, and a
# test that doesn't.
cc_googletest(
    name = "mixed_integer_optimization_test",
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
    ],
)

cc_googletest(
    name = "moby_lcp_solver_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "system_identification_test",
    # The test takes ~20 sec in opt mode but ~140 sec in dbg mode.
    # To avoid timeouts in dbg or valgrind, we need "moderate" here.
    # TODO(bazelbuild/bazel#1748) This produces a spurious complaint.
    timeout = "moderate",
    deps = [
        ":system_identification",
        "//drake/common:eigen_matrix_compare",
    ],
)
