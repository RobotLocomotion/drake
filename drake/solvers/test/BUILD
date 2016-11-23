# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["@gtest//:main", "//drake/common:eigen_matrix_compare"]

cc_test(
    name = "constraint_test",
    srcs = ["constraint_test.cc"],
    deps = DEPS + [
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
    size = "small")

cc_test(
    name = "convex_optimization_test",
    srcs = [
        "convex_optimization_test.cc",
        "mathematical_program_test_util.h",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
    size = "small")

cc_test(
    name = "fastqp_solver_test",
    srcs = ["fastqp_solver_test.cc"],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
    size = "small")

cc_test(
    name = "mathematical_program_test",
    srcs = [
        "mathematical_program_test.cc",
        "mathematical_program_test_util.h",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
    size = "small")

cc_test(
    name = "mixed_integer_optimization_test",
    srcs = [
        "mathematical_program_test_util.h",
        "mixed_integer_optimization_test.cc",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
    size = "small")

cc_test(
    name = "moby_lcp_solver_test",
    srcs = ["moby_lcp_solver_test.cc"],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
    size = "small")

cc_test(
    name = "system_identification_test",
    srcs = ["system_identification_test.cc"],
    deps = DEPS + ["//drake/solvers:system_identification"],
    size = "medium")
