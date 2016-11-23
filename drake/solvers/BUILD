# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/common",
    "//drake/common:autodiff",
    "//drake/common:polynomial",
]

# TODO(jwnimmer-tri) This BUILD file only compiles the in-tree solvers.
# External solvers are not yet supported.

cc_library(
    name = "mathematical_program",
    srcs = [
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
        "function.h",
        "constraint.h",
        "decision_variable.h",
        "mathematical_program.h",
        "solution_result.h",
    ],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "system_identification",
    srcs = ["system_identification.cc"],
    hdrs = ["system_identification.h"],
    deps = DEPS + [":mathematical_program"],
    linkstatic = 1)
