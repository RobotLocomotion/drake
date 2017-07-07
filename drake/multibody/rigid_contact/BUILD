# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rigid_contact",
    hdrs = [
        "rigid_contact_problem_data.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "rigid_contact_solver",
    srcs = ["rigid_contact_solver.cc"],
    hdrs = [
        "rigid_contact_solver.h",
    ],
    deps = [
        "//drake/solvers:mathematical_program",
        "rigid_contact",
    ],
)


# === test/ ===

drake_cc_googletest(
    name = "rigid_contact_solver_test",
    deps = [
        ":rigid_contact_solver",
        "//drake/examples/rod2d",
    ],
)

cpplint()
