# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "@gtest//:main",
    "//drake/common:eigen_matrix_compare",
]

cc_test(
    name = "constraint_test",
    size = "small",
    srcs = ["constraint_test.cc"],
    deps = DEPS + [
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

cc_test(
    name = "convex_optimization_test",
    size = "small",
    srcs = [
        "convex_optimization_test.cc",
        "mathematical_program_test_util.h",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
)

cc_test(
    name = "decision_variable_test",
    size = "small",
    srcs = ["decision_variable_test.cc"],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
)

cc_test(
    name = "mathematical_program_test",
    size = "small",
    srcs = [
        "mathematical_program_test.cc",
        "mathematical_program_test_util.h",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
)

cc_test(
    name = "mixed_integer_optimization_test",
    size = "small",
    srcs = [
        "mathematical_program_test_util.h",
        "mixed_integer_optimization_test.cc",
    ],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
)

cc_test(
    name = "moby_lcp_solver_test",
    size = "small",
    srcs = ["moby_lcp_solver_test.cc"],
    deps = DEPS + ["//drake/solvers:mathematical_program"],
)

cc_test(
    name = "system_identification_test",
    size = "medium",
    srcs = ["system_identification_test.cc"],
    deps = DEPS + ["//drake/solvers:system_identification"],
)
