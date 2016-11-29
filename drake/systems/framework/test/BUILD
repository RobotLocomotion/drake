# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "leaf_context_test",
    size = "small",
    testonly = 1,
    srcs = ["leaf_context_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "modal_state_test",
    size = "small",
    testonly = 1,
    srcs = ["modal_state_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cache_test",
    size = "small",
    testonly = 1,
    srcs = ["cache_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "parameters_test",
    size = "small",
    testonly = 1,
    srcs = ["parameters_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_test",
    size = "small",
    testonly = 1,
    srcs = ["diagram_test.cc"],
    linkstatic = 1,
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
)

cc_test(
    name = "diagram_context_test",
    size = "small",
    testonly = 1,
    srcs = ["diagram_context_test.cc"],
    linkstatic = 1,
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
)
