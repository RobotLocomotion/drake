# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "analysis",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    deps = [
        "//drake/common:extract_double",
        "//drake/systems/framework",
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
        "@gtest//:main",
    ],
)

cc_test(
    name = "simulator_test",
    size = "small",
    srcs = ["test/simulator_test.cc"],
    linkstatic = 1,
    deps = [
        ":analysis",
        ":my_spring_mass_system",
        "//drake/systems/analysis/test/controlled_spring_mass_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "explicit_euler_integrator_test",
    size = "small",
    srcs = ["test/explicit_euler_integrator_test.cc"],
    linkstatic = 1,
    deps = [
        ":analysis",
        ":my_spring_mass_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "runge_kutta2_integrator_test",
    size = "small",
    srcs = ["test/runge_kutta2_integrator_test.cc"],
    linkstatic = 1,
    deps = [
        ":analysis",
        ":my_spring_mass_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "runge_kutta3_integrator_test",
    size = "small",
    srcs = ["test/runge_kutta3_integrator_test.cc"],
    linkstatic = 1,
    deps = [
        ":analysis",
        ":my_spring_mass_system",
        "//drake/common:drake_path",
        "//drake/math:geometric_transform",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/parsers:multi_body_parsers",
        "@gtest//:main",
    ],
)
