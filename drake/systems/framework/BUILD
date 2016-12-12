# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_unit_test")

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

cc_unit_test(
    name = "basic_vector_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:functional_form",
    ],
)

cc_unit_test(
    name = "cache_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_unit_test(
    name = "continuous_state_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "diagram_builder_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:demultiplexer",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
    ],
)

cc_unit_test(
    name = "diagram_context_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:zero_order_hold",
    ],
)

cc_unit_test(
    name = "diagram_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:zero_order_hold",
    ],
)

cc_unit_test(
    name = "discrete_state_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "leaf_context_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_unit_test(
    name = "leaf_system_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "abstract_state_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_unit_test(
    name = "parameters_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_unit_test(
    name = "subvector_test",
    deps = [
        ":framework",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "supervector_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "system_input_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "system_output_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "system_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)

cc_unit_test(
    name = "value_test",
    deps = [
        ":framework",
        "//drake/common",
    ],
)
