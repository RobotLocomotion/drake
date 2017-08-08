# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "hard_constraint",
    hdrs = [
        "hard_constraint_problem_data.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "hard_constraint_solver",
    srcs = ["hard_constraint_solver.cc"],
    hdrs = [
        "hard_constraint_solver.h",
    ],
    deps = [
        ":hard_constraint",
        "//drake/solvers:mathematical_program",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "hard_constraint_solver_test",
    deps = [
        ":hard_constraint_solver",
        "//drake/examples/rod2d",
    ],
)

add_lint_tests()
