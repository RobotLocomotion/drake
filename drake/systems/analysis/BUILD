# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "analysis",
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":explicit_euler_integrator",
        ":runge_kutta2_integrator",
        ":runge_kutta3_integrator",
        ":simulator",
    ],
)

cc_library(
    name = "integrator_base",
    hdrs = ["integrator_base.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework:context",
        "//drake/systems/framework:system",
    ],
)

cc_library(
    name = "explicit_euler_integrator",
    srcs = [],
    hdrs = ["explicit_euler_integrator.h"],
    linkstatic = 1,
    deps = [
        ":integrator_base",
    ],
)

cc_library(
    name = "runge_kutta2_integrator",
    srcs = [],
    hdrs = ["runge_kutta2_integrator.h"],
    linkstatic = 1,
    deps = [
        ":integrator_base",
    ],
)

cc_library(
    name = "runge_kutta3_integrator",
    srcs = ["runge_kutta3_integrator.cc"],
    hdrs = [
        "runge_kutta3_integrator.h",
        "runge_kutta3_integrator-inl.h",
    ],
    linkstatic = 1,
    deps = [
        ":integrator_base",
        ":runge_kutta2_integrator",
    ],
)

cc_library(
    name = "simulator",
    srcs = ["simulator.cc"],
    hdrs = ["simulator.h"],
    linkstatic = 1,
    deps = [
        ":runge_kutta2_integrator",
        "//drake/common:extract_double",
        "//drake/systems/framework:context",
        "//drake/systems/framework:system",
    ],
)

# === test/ ===

cc_library(
    name = "my_spring_mass_system",
    testonly = 1,
    hdrs = ["test/my_spring_mass_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/plants/spring_mass_system",
    ],
)

cc_googletest(
    name = "simulator_test",
    deps = [
        ":explicit_euler_integrator",
        ":my_spring_mass_system",
        ":simulator",
        "//drake/systems/analysis/test/controlled_spring_mass_system",
    ],
)

cc_googletest(
    name = "explicit_euler_integrator_test",
    deps = [
        ":explicit_euler_integrator",
        ":my_spring_mass_system",
    ],
)

cc_googletest(
    name = "runge_kutta2_integrator_test",
    deps = [
        ":my_spring_mass_system",
        ":runge_kutta2_integrator",
    ],
)

cc_googletest(
    name = "runge_kutta3_integrator_test",
    deps = [
        ":my_spring_mass_system",
        ":runge_kutta3_integrator",
        "//drake/common:drake_path",
        "//drake/math:geometric_transform",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)
