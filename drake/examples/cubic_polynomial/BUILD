# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_binary")
load("//tools:lint.bzl", "add_lint_tests")
load("//tools:mosek.bzl", "mosek_test_tags")

drake_cc_binary(
    name = "region_of_attraction",
    srcs = ["region_of_attraction.cc"],
    add_test_rule = 1,
    tags = mosek_test_tags(),
    deps = [
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework:vector_system",
    ],
)

add_lint_tests()
