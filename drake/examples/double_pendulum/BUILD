# -*- python -*-

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

filegroup(
    name = "models",
    srcs = glob([
        "models/*.sdf",
    ]),
)

drake_cc_library(
    name = "sdf_helpers",
    srcs = [
        "frame_cache.cc",
        "sdf_helpers.cc",
    ],
    hdrs = [
        "frame_cache.h",
        "sdf_helpers.h",
    ],
    deps = [
        "//drake/common",
        "//drake/multibody:rigid_body_tree",
        "@eigen",
        "@sdformat",
    ],
)

drake_cc_binary(
    name = "double_pendulum_demo",
    srcs = [
        "double_pendulum_demo.cc",
    ],
    data = [
        ":models",
        "//tools:drake_visualizer",
    ],
    deps = [
        ":sdf_helpers",
        "//drake/common",
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "@sdformat",
    ],
)

# === test/ ===

drake_cc_library(
    name = "rigid_body_types_compare",
    testonly = 1,
    srcs = [
        "test/rigid_body_types_compare.cc",
    ],
    hdrs = [
        "test/rigid_body_types_compare.h",
    ],
    deps = [
        "//drake/multibody:rigid_body",
        "//drake/multibody/shapes",
        "@gtest//:without_main",
    ],
)

drake_cc_googletest(
    name = "frame_cache_test",
    deps = [
        ":sdf_helpers",
    ],
)

drake_cc_googletest(
    name = "sdf_helpers_test",
    data = [
        ":models",
    ],
    deps = [
        ":rigid_body_types_compare",
        ":sdf_helpers",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
    ],
)

add_lint_tests()
