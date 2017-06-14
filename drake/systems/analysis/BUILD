# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "analysis",
    srcs = [],
    hdrs = [],
    deps = [
        ":explicit_euler_integrator",
        ":implicit_euler_integrator",
        ":runge_kutta2_integrator",
        ":runge_kutta3_integrator",
        ":semi_explicit_euler_integrator",
        ":simulator",
    ],
)

drake_cc_library(
    name = "integrator_base",
    hdrs = ["integrator_base.h"],
    deps = [
        "//drake/systems/framework:context",
        "//drake/systems/framework:system",
    ],
)

drake_cc_library(
    name = "explicit_euler_integrator",
    srcs = [],
    hdrs = ["explicit_euler_integrator.h"],
    deps = [
        ":integrator_base",
    ],
)

drake_cc_library(
    name = "runge_kutta2_integrator",
    srcs = [],
    hdrs = ["runge_kutta2_integrator.h"],
    deps = [
        ":integrator_base",
    ],
)

drake_cc_library(
    name = "implicit_euler_integrator",
    srcs = ["implicit_euler_integrator.cc"],
    hdrs = [
        "implicit_euler_integrator.h",
        "implicit_euler_integrator-inl.h",
    ],
    deps = [
        ":integrator_base",
        "//drake/math:gradient",
    ],
)

drake_cc_library(
    name = "runge_kutta3_integrator",
    srcs = ["runge_kutta3_integrator.cc"],
    hdrs = [
        "runge_kutta3_integrator.h",
        "runge_kutta3_integrator-inl.h",
    ],
    deps = [
        ":integrator_base",
        ":runge_kutta2_integrator",
    ],
)

drake_cc_library(
    name = "semi_explicit_euler_integrator",
    srcs = [],
    hdrs = [
        "semi_explicit_euler_integrator.h",
    ],
    deps = [
        ":integrator_base",
    ],
)

drake_cc_library(
    name = "simulator",
    srcs = ["simulator.cc"],
    hdrs = ["simulator.h"],
    deps = [
        ":runge_kutta2_integrator",
        ":runge_kutta3_integrator",
        "//drake/common:extract_double",
        "//drake/systems/framework:context",
        "//drake/systems/framework:system",
    ],
)

# === test/ ===

drake_cc_library(
    name = "robertson_system",
    testonly = 1,
    hdrs = ["test/robertson_system.h"],
    deps = [],
)

drake_cc_library(
    name = "stiff_double_mass_spring_system",
    testonly = 1,
    hdrs = ["test/stiff_double_mass_spring_system.h"],
    deps = [],
)

drake_cc_library(
    name = "spring_mass_damper_system",
    testonly = 1,
    hdrs = ["test/spring_mass_damper_system.h"],
    deps = [
        "//drake/systems/plants/spring_mass_system",
    ],
)

drake_cc_library(
    name = "discontinuous_spring_mass_damper_system",
    testonly = 1,
    hdrs = ["test/discontinuous_spring_mass_damper_system.h"],
    deps = [":spring_mass_damper_system"],
)

drake_cc_library(
    name = "my_spring_mass_system",
    testonly = 1,
    hdrs = ["test/my_spring_mass_system.h"],
    deps = [
        "//drake/systems/plants/spring_mass_system",
    ],
)

drake_cc_library(
    name = "logistic_system",
    testonly = 1,
    hdrs = ["test/logistic_system.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "empty_system",
    testonly = 1,
    hdrs = ["test/empty_system.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "simulator_test",
    deps = [
        ":empty_system",
        ":explicit_euler_integrator",
        ":implicit_euler_integrator",
        ":logistic_system",
        ":my_spring_mass_system",
        ":runge_kutta3_integrator",
        ":simulator",
        "//drake/common:is_dynamic_castable",
        "//drake/systems/analysis/test/controlled_spring_mass_system",
    ],
)

drake_cc_googletest(
    name = "explicit_euler_integrator_test",
    deps = [
        ":explicit_euler_integrator",
        ":my_spring_mass_system",
    ],
)

drake_cc_googletest(
    name = "implicit_euler_integrator_test",
    # Test size increased to not timeout when run with Valgrind.
    size = "medium",
    deps = [
        ":discontinuous_spring_mass_damper_system",
        ":implicit_euler_integrator",
        ":robertson_system",
        ":spring_mass_damper_system",
        ":stiff_double_mass_spring_system",
        "//drake/systems/plants/spring_mass_system",
    ],
)

drake_cc_googletest(
    name = "runge_kutta2_integrator_test",
    deps = [
        ":my_spring_mass_system",
        ":runge_kutta2_integrator",
    ],
)

drake_cc_googletest(
    name = "runge_kutta3_integrator_test",
    # Test size increased to not timeout when run with Valgrind.
    size = "medium",
    deps = [
        ":my_spring_mass_system",
        ":runge_kutta3_integrator",
        "//drake/common:drake_path",
        "//drake/math:geometric_transform",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "semi_explicit_euler_integrator_test",
    # Test size increased to not timeout when run with Valgrind.
    size = "medium",
    deps = [
        ":explicit_euler_integrator",
        ":my_spring_mass_system",
        ":semi_explicit_euler_integrator",
        "//drake/multibody/joints",
        "//drake/multibody/rigid_body_plant",
    ],
)

cpplint()
