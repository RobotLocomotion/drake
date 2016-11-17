# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "leaf_context_test",
    srcs = ["leaf_context_test.cc"],
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)

cc_test(
    name = "modal_state_test",
    srcs = ["modal_state_test.cc"],
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)

cc_test(
    name = "cache_test",
    srcs = ["cache_test.cc"],
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)

cc_test(
    name = "parameters_test",
    srcs = ["parameters_test.cc"],
    deps = [
        "//drake/common", 
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)

cc_test(
    name = "diagram_test",
    srcs = ["diagram_test.cc"],
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:zero_order_hold",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)

cc_test(
    name = "diagram_context_test",
    srcs = ["diagram_context_test.cc"],
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:zero_order_hold",
        "@gtest//:main",
    ],
    size = "small",
    linkstatic=1,
    testonly=1)
