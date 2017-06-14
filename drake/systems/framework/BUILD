# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "framework",
    srcs = [],
    hdrs = ["systems.h"],
    deps = [
        ":cache",
        ":context",
        ":diagram",
        ":diagram_builder",
        ":diagram_context",
        ":leaf_context",
        ":leaf_system",
        ":parameters",
        ":single_output_vector_source",
        ":siso_vector_system",
        ":sparsity_matrix",
        ":state",
        ":system",
        ":value",
        ":vector",
    ],
)

drake_cc_library(
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
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:dummy_value",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "value",
    srcs = ["value.cc"],
    hdrs = ["value.h"],
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
        "//drake/common:is_cloneable",
    ],
)

drake_cc_library(
    name = "value_checker",
    srcs = [],
    hdrs = ["value_checker.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":value",
        "//drake/common",
        "//drake/common:nice_type_name",
    ],
)

drake_cc_library(
    name = "cache",
    srcs = ["cache.cc"],
    hdrs = ["cache.h"],
    deps = [
        ":value",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "abstract_values",
    srcs = ["abstract_values.cc"],
    hdrs = ["abstract_values.h"],
    deps = [
        ":value",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "continuous_state",
    srcs = ["continuous_state.cc"],
    hdrs = ["continuous_state.h"],
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "discrete_values",
    srcs = ["discrete_values.cc"],
    hdrs = ["discrete_values.h"],
    deps = [
        ":value",
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    deps = [
        ":abstract_values",
        ":continuous_state",
        ":discrete_values",
        ":value",
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "parameters",
    srcs = ["parameters.cc"],
    hdrs = ["parameters.h"],
    deps = [
        ":abstract_values",
        ":discrete_values",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "system_common",
    srcs = [],
    hdrs = [
        "system_common.h",
    ],
    deps = [
    ],
)

drake_cc_library(
    name = "input_port_descriptor",
    srcs = [
        "input_port_descriptor.cc",
    ],
    hdrs = [
        "input_port_descriptor.h",
    ],
    deps = [
        ":system_common",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "output_port_listener_interface",
    srcs = ["output_port_listener_interface.cc"],
    hdrs = ["output_port_listener_interface.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "output_port_value",
    srcs = ["output_port_value.cc"],
    hdrs = ["output_port_value.h"],
    deps = [
        ":output_port_listener_interface",
        ":value",
        ":value_checker",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "input_port_evaluator_interface",
    srcs = ["input_port_evaluator_interface.cc"],
    hdrs = ["input_port_evaluator_interface.h"],
    deps = [
        ":input_port_descriptor",
        ":output_port_value",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "input_port_value",
    srcs = ["input_port_value.cc"],
    hdrs = ["input_port_value.h"],
    deps = [
        ":input_port_evaluator_interface",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "context",
    srcs = ["context.cc"],
    hdrs = ["context.h"],
    deps = [
        ":input_port_evaluator_interface",
        ":input_port_value",
        ":parameters",
        ":state",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "leaf_context",
    srcs = ["leaf_context.cc"],
    hdrs = ["leaf_context.h"],
    deps = [
        ":cache",
        ":context",
        ":parameters",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "system",
    srcs = [
        "output_port.cc",
        "system.cc",
    ],
    hdrs = [
        "discrete_event.h",
        "output_port.h",
        "system.h",
        "witness_function.h",
    ],
    deps = [
        ":cache",
        ":context",
        ":system_common",
        ":value",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:symbolic",
        "//drake/common:type_safe_index",
        "//drake/common:unused",
    ],
)

drake_cc_library(
    name = "leaf_output_port",
    srcs = ["leaf_output_port.cc"],
    hdrs = ["leaf_output_port.h"],
    deps = [
        ":system",
        ":value",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "model_values",
    srcs = ["model_values.cc"],
    hdrs = ["model_values.h"],
    visibility = [":__subpackages__"],
    deps = [
        ":value",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "leaf_system",
    srcs = ["leaf_system.cc"],
    hdrs = ["leaf_system.h"],
    deps = [
        ":leaf_context",
        ":leaf_output_port",
        ":model_values",
        ":sparsity_matrix",
        ":system",
        ":value_checker",
        "//drake/common",
        "//drake/common:number_traits",
        "//drake/common:unused",
    ],
)

drake_cc_library(
    name = "diagram_continuous_state",
    srcs = ["diagram_continuous_state.cc"],
    hdrs = ["diagram_continuous_state.h"],
    deps = [
        ":state",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "diagram_context",
    srcs = ["diagram_context.cc"],
    hdrs = ["diagram_context.h"],
    deps = [
        ":context",
        ":diagram_continuous_state",
        ":vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "diagram",
    srcs = ["diagram.cc"],
    hdrs = ["diagram.h"],
    deps = [
        ":cache",
        ":diagram_context",
        ":system",
        "//drake/common",
        "//drake/common:number_traits",
    ],
)

drake_cc_library(
    name = "diagram_builder",
    srcs = ["diagram_builder.cc"],
    hdrs = ["diagram_builder.h"],
    deps = [
        ":diagram",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "sparsity_matrix",
    srcs = ["sparsity_matrix.cc"],
    hdrs = ["sparsity_matrix.h"],
    deps = [
        ":context",
        ":system",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "single_output_vector_source",
    srcs = ["single_output_vector_source.cc"],
    hdrs = ["single_output_vector_source.h"],
    deps = [
        ":leaf_system",
    ],
)

drake_cc_library(
    name = "siso_vector_system",
    srcs = ["siso_vector_system.cc"],
    hdrs = ["siso_vector_system.h"],
    deps = [
        ":leaf_system",
        "//drake/common:unused",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "basic_vector_test",
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:symbolic",
    ],
)

drake_cc_googletest(
    name = "cache_test",
    deps = [
        ":cache",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "continuous_state_test",
    deps = [
        ":continuous_state",
        "//drake/common",
    ],
)

drake_cc_googletest(
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

drake_cc_googletest(
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

drake_cc_googletest(
    name = "diagram_test",
    deps = [
        ":diagram",
        "//drake/common",
        "//drake/systems/framework/test_utilities:pack_value",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:zero_order_hold",
    ],
)

drake_cc_googletest(
    name = "discrete_values_test",
    deps = [
        ":discrete_values",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "leaf_context_test",
    deps = [
        ":leaf_context",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:is_dynamic_castable",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "leaf_system_test",
    deps = [
        ":leaf_system",
        "//drake/common",
        "//drake/common:is_dynamic_castable",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "model_values_test",
    deps = [
        ":model_values",
        "//drake/common:autodiff",
        "//drake/common:symbolic",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "abstract_values_test",
    deps = [
        ":abstract_values",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "parameters_test",
    deps = [
        ":parameters",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "subvector_test",
    deps = [
        ":vector",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "supervector_test",
    deps = [
        ":vector",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "input_port_value_test",
    deps = [
        ":input_port_value",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "output_port_value_test",
    deps = [
        ":output_port_value",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "output_port_test",
    deps = [
        ":leaf_context",
        ":system",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_googletest(
    name = "system_test",
    deps = [
        ":leaf_context",
        ":leaf_output_port",
        ":system",
        "//drake/common",
        "//drake/systems/framework/test_utilities",
    ],
)

drake_cc_googletest(
    name = "value_test",
    deps = [
        ":value",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "value_checker_test",
    deps = [
        ":value_checker",
    ],
)

drake_cc_googletest(
    name = "sparsity_matrix_test",
    deps = [
        ":leaf_system",
        ":sparsity_matrix",
    ],
)

drake_cc_googletest(
    name = "single_output_vector_source_test",
    deps = [
        ":single_output_vector_source",
    ],
)

drake_cc_googletest(
    name = "siso_vector_system_test",
    deps = [
        ":siso_vector_system",
    ],
)

cpplint()
