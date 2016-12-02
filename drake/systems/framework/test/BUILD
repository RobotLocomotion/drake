# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "basic_vector_test",
    size = "small",
    srcs = ["basic_vector_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:functional_form",
        "//drake/systems/framework",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cache_test",
    size = "small",
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
    name = "continuous_state_test",
    size = "small",
    srcs = ["continuous_state_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_builder_test",
    size = "small",
    srcs = ["diagram_builder_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:demultiplexer",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_context_test",
    size = "small",
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

cc_test(
    name = "diagram_test",
    size = "small",
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
    name = "difference_state_test",
    size = "small",
    srcs = ["difference_state_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "leaf_context_test",
    size = "small",
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
    name = "leaf_system_test",
    size = "small",
    srcs = ["leaf_system_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "modal_state_test",
    size = "small",
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
    name = "parameters_test",
    size = "small",
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
    name = "subvector_test",
    size = "small",
    srcs = ["subvector_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "supervector_test",
    size = "small",
    srcs = ["supervector_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_input_test",
    size = "small",
    srcs = ["system_input_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_output_test",
    size = "small",
    srcs = ["system_output_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_test",
    size = "small",
    srcs = ["system_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)

cc_test(
    name = "value_test",
    size = "small",
    srcs = ["value_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework",
        "@gtest//:main",
    ],
)
