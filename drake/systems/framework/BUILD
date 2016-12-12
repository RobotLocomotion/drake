# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "framework",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:number_traits",
    ],
)

# === test/ ===

cc_test(
    name = "basic_vector_test",
    size = "small",
    testonly = 1,
    srcs = ["test/basic_vector_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:functional_form",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cache_test",
    size = "small",
    testonly = 1,
    srcs = ["test/cache_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "continuous_state_test",
    size = "small",
    testonly = 1,
    srcs = ["test/continuous_state_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_builder_test",
    size = "small",
    testonly = 1,
    srcs = ["test/diagram_builder_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_context_test",
    size = "small",
    testonly = 1,
    srcs = ["test/diagram_context_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:zero_order_hold",
        "@gtest//:main",
    ],
)

cc_test(
    name = "diagram_test",
    size = "small",
    testonly = 1,
    srcs = ["test/diagram_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:zero_order_hold",
        "@gtest//:main",
    ],
)

cc_test(
    name = "discrete_state_test",
    size = "small",
    testonly = 1,
    srcs = ["test/discrete_state_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "leaf_context_test",
    size = "small",
    testonly = 1,
    srcs = ["test/leaf_context_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "leaf_system_test",
    size = "small",
    testonly = 1,
    srcs = ["test/leaf_system_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "abstract_state_test",
    size = "small",
    testonly = 1,
    srcs = ["test/abstract_state_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "parameters_test",
    size = "small",
    testonly = 1,
    srcs = ["test/parameters_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
        "@gtest//:main",
    ],
)

cc_test(
    name = "subvector_test",
    size = "small",
    testonly = 1,
    srcs = ["test/subvector_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "supervector_test",
    size = "small",
    testonly = 1,
    srcs = ["test/supervector_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_input_test",
    size = "small",
    testonly = 1,
    srcs = ["test/system_input_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_output_test",
    size = "small",
    testonly = 1,
    srcs = ["test/system_output_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "system_test",
    size = "small",
    testonly = 1,
    srcs = ["test/system_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "value_test",
    size = "small",
    testonly = 1,
    srcs = ["test/value_test.cc"],
    linkstatic = 1,
    deps = [
        ":framework",
        "//drake/common",
        "@gtest//:main",
    ],
)
