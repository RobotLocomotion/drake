# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "framework",
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":cache",
        ":context",
        ":diagram",
        ":diagram_builder",
        ":diagram_context",
        ":leaf_context",
        ":leaf_system",
        ":parameters",
        ":state",
        ":system",
        ":value",
        ":vector",
    ],
)

cc_library(
    name = "vector",
    srcs = [
        "basic_vector.cc",
        "subvector.cc",
        "supervector.cc",
        "vector_base.cc",
    ],
    hdrs = [
        "basic_vector.h",
        "subvector.h",
        "supervector.h",
        "vector_base.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "value",
    srcs = ["value.cc"],
    hdrs = ["value.h"],
    linkstatic = 1,
    deps = [
        ":vector",
        "//drake/common",
    ],
)

cc_library(
    name = "cache",
    srcs = ["cache.cc"],
    hdrs = ["cache.h"],
    linkstatic = 1,
    deps = [
        ":value",
        "//drake/common",
    ],
)

cc_library(
    name = "abstract_state",
    srcs = ["abstract_state.cc"],
    hdrs = ["abstract_state.h"],
    linkstatic = 1,
    deps = [
        ":value",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "continuous_state",
    srcs = ["continuous_state.cc"],
    hdrs = ["continuous_state.h"],
    linkstatic = 1,
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "discrete_state",
    srcs = ["discrete_state.cc"],
    hdrs = ["discrete_state.h"],
    linkstatic = 1,
    deps = [
        ":value",
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    linkstatic = 1,
    deps = [
        ":abstract_state",
        ":continuous_state",
        ":discrete_state",
        ":value",
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "parameters",
    srcs = ["parameters.cc"],
    hdrs = ["parameters.h"],
    linkstatic = 1,
    deps = [
        ":abstract_state",
        ":discrete_state",
        "//drake/common",
    ],
)

cc_library(
    name = "system_port_descriptor",
    srcs = ["system_port_descriptor.cc"],
    hdrs = ["system_port_descriptor.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "output_port_listener_interface",
    srcs = ["output_port_listener_interface.cc"],
    hdrs = ["output_port_listener_interface.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "system_output",
    srcs = ["system_output.cc"],
    hdrs = ["system_output.h"],
    linkstatic = 1,
    deps = [
        ":output_port_listener_interface",
        ":value",
        ":vector",
        "//drake/common",
    ],
)

cc_library(
    name = "input_port_evaluator_interface",
    srcs = ["input_port_evaluator_interface.cc"],
    hdrs = ["input_port_evaluator_interface.h"],
    linkstatic = 1,
    deps = [
        ":system_output",
        ":system_port_descriptor",
        "//drake/common",
    ],
)

cc_library(
    name = "system_input",
    srcs = ["system_input.cc"],
    hdrs = ["system_input.h"],
    linkstatic = 1,
    deps = [
        ":input_port_evaluator_interface",
        ":vector",
        "//drake/common",
    ],
)

cc_library(
    name = "context",
    srcs = ["context.cc"],
    hdrs = ["context.h"],
    linkstatic = 1,
    deps = [
        ":input_port_evaluator_interface",
        ":state",
        ":system_input",
        "//drake/common",
    ],
)

cc_library(
    name = "leaf_context",
    srcs = ["leaf_context.cc"],
    hdrs = ["leaf_context.h"],
    linkstatic = 1,
    deps = [
        ":cache",
        ":context",
        ":parameters",
        ":vector",
        "//drake/common",
    ],
)

cc_library(
    name = "system",
    srcs = ["system.cc"],
    hdrs = ["system.h"],
    linkstatic = 1,
    deps = [
        ":cache",
        ":context",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "leaf_system",
    srcs = ["leaf_system.cc"],
    hdrs = ["leaf_system.h"],
    linkstatic = 1,
    deps = [
        ":leaf_context",
        ":system",
        "//drake/common",
        "//drake/common:number_traits",
    ],
)

cc_library(
    name = "diagram_continuous_state",
    srcs = ["diagram_continuous_state.cc"],
    hdrs = ["diagram_continuous_state.h"],
    linkstatic = 1,
    deps = [
        ":state",
        "//drake/common",
    ],
)

cc_library(
    name = "diagram_context",
    srcs = ["diagram_context.cc"],
    hdrs = ["diagram_context.h"],
    linkstatic = 1,
    deps = [
        ":context",
        ":diagram_continuous_state",
        ":vector",
        "//drake/common",
    ],
)

cc_library(
    name = "diagram",
    srcs = ["diagram.cc"],
    hdrs = ["diagram.h"],
    linkstatic = 1,
    deps = [
        ":cache",
        ":diagram_context",
        ":system",
        "//drake/common",
        "//drake/common:number_traits",
    ],
)

cc_library(
    name = "diagram_builder",
    srcs = ["diagram_builder.cc"],
    hdrs = ["diagram_builder.h"],
    linkstatic = 1,
    deps = [
        ":diagram",
        "//drake/common",
    ],
)

# === test/ ===

cc_googletest(
    name = "basic_vector_test",
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:functional_form",
    ],
)

cc_googletest(
    name = "cache_test",
    deps = [
        ":cache",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_googletest(
    name = "continuous_state_test",
    deps = [
        ":continuous_state",
        "//drake/common",
    ],
)

cc_googletest(
    name = "diagram_builder_test",
    deps = [
        ":diagram_builder",
        "//drake/common",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
    ],
)

cc_googletest(
    name = "diagram_context_test",
    deps = [
        ":diagram_context",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework/test_utilities:pack_value",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:zero_order_hold",
    ],
)

cc_googletest(
    name = "diagram_test",
    deps = [
        ":diagram",
        "//drake/common",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:zero_order_hold",
    ],
)

cc_googletest(
    name = "discrete_state_test",
    deps = [
        ":discrete_state",
        "//drake/common",
    ],
)

cc_googletest(
    name = "leaf_context_test",
    deps = [
        ":leaf_context",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_googletest(
    name = "leaf_system_test",
    deps = [
        ":leaf_system",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_googletest(
    name = "abstract_state_test",
    deps = [
        ":abstract_state",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_googletest(
    name = "parameters_test",
    deps = [
        ":parameters",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

cc_googletest(
    name = "subvector_test",
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "supervector_test",
    deps = [
        ":vector",
        "//drake/common",
    ],
)

cc_googletest(
    name = "system_input_test",
    deps = [
        ":system_input",
        "//drake/common",
    ],
)

cc_googletest(
    name = "system_output_test",
    deps = [
        ":system_output",
        "//drake/common",
    ],
)

cc_googletest(
    name = "system_test",
    deps = [
        ":leaf_context",
        ":system",
        "//drake/common",
    ],
)

cc_googletest(
    name = "value_test",
    deps = [
        ":value",
        "//drake/common",
    ],
)
