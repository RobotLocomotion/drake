# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "binding",
    srcs = [],
    hdrs = ["binding.h"],
    linkstatic = 1,
    deps = [
        ":decision_variable",
    ],
)

drake_cc_library(
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

drake_cc_library(
    name = "decision_variable",
    srcs = ["decision_variable.cc"],
    hdrs = ["decision_variable.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:number_traits",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
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
# - IPOPT
# - SNOPT
# - Gurobi (on Ubuntu only)

# Unsupported solvers:
# - dReal
# - NLOPT
# - MOSEK

# TODO(jwnimmer-tri, david-german-tri): Support more solvers.
drake_cc_library(
    name = "mathematical_program",
    srcs = [
        "equality_constrained_qp_solver.cc",
        "linear_system_solver.cc",
        "mathematical_program.cc",
        "mathematical_program_solver_interface.cc",
        "moby_lcp_solver.cc",
        "no_dreal.cc",
        "no_mosek.cc",
        "no_nlopt.cc",
    ],
    hdrs = [
        "dreal_solver.h",
        "equality_constrained_qp_solver.h",
        "linear_system_solver.h",
        "mathematical_program.h",
        "mathematical_program_solver_interface.h",
        "moby_lcp_solver.h",
        "mosek_solver.h",
        "nlopt_solver.h",
        "snopt_solver.h",
    ],
    linkstatic = 1,
    deps = [
        ":binding",
        ":constraint",
        ":decision_variable",
        ":function",
        ":gurobi_solver",
        ":ipopt_solver",
        ":snopt_solver",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:number_traits",
        "//drake/common:polynomial",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
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
drake_cc_library(
    name = "mathematical_program_api",
    srcs = [],
    hdrs = [
        "mathematical_program.h",
        "mathematical_program_solver_interface.h",
    ],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":binding",
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
drake_cc_library(
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

# If --config snopt is used, this target builds object code that satisfies
# MathematicalProgramSolverInterface by calling SNOPT. Otherwise, this target
# builds dummy object code that fails at runtime.
drake_cc_library(
    name = "snopt_solver",
    srcs = select({
        "//tools:with_snopt": ["snopt_solver.cc"],
        "//conditions:default": ["no_snopt.cc"],
    }),
    hdrs = ["snopt_solver.h"],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = select({
        "//tools:with_snopt": [
            ":mathematical_program_api",
            "//drake/math:autodiff",
            "@snopt//:snopt_c",
        ],
        "//conditions:default": [
            ":mathematical_program_api",
        ],
    }),
)

drake_cc_library(
    name = "ipopt_solver",
    srcs = ["ipopt_solver.cc"],
    hdrs = ["ipopt_solver.h"],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":mathematical_program_api",
        "//drake/math:autodiff",
        "@ipopt//:lib",
    ],
)

# === test/ ===
drake_cc_googletest(
    name = "binding_test",
    srcs = [
        "test/binding_test.cc",
    ],
    deps = [
        ":binding",
        ":constraint",
    ],
)

drake_cc_googletest(
    name = "constraint_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:gradient",
    ],
)

# TODO(hongkai-dai): Separate this test into a test that uses Gurobi, and a
# test that doesn't.
drake_cc_googletest(
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

drake_cc_googletest(
    name = "decision_variable_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "equality_constrained_qp_solver_test",
    srcs = [
        "test/equality_constrained_qp_solver_test.cc",
        "test/mathematical_program_test_util.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "linear_complementary_problem_test",
    srcs = [
        "test/linear_complementary_problem_test.cc",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "linear_system_solver_test",
    srcs = [
        "test/linear_system_solver_test.cc",
        "test/mathematical_program_test_util.h",
        "test/optimization_examples.cc",
        "test/optimization_examples.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
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
drake_cc_googletest(
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

drake_cc_googletest(
    name = "moby_lcp_solver_test",
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "nonlinear_program_test",
    srcs = [
        "test/mathematical_program_test_util.h",
        "test/nonlinear_program_test.cc",
        "test/optimization_examples.cc",
        "test/optimization_examples.h",
    ],
    deps = [
        ":mathematical_program",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
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

# The extra_srcs are required here because cpplint() doesn't understand how to
# extract labels from select() functions yet.
cpplint(extra_srcs = [
    "gurobi_solver.cc",
    "gurobi_solver.h",
    "no_gurobi.cc",
    "snopt_solver.cc",
    "snopt_solver.h",
    "no_snopt.cc",
])
