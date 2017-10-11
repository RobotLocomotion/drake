# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "traffic_light",
    srcs = [
        "traffic_light.cc",
    ],
    hdrs = [
        "traffic_light.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "traffic_light_aware_controller",
    srcs = [
        "traffic_light_aware_controller.cc",
    ],
    hdrs = [
        "traffic_light_aware_controller.h",
    ],
    deps = [
        "//drake/automotive:generated_vectors",
        "//drake/systems/framework",
    ],
)

# === test/ ===
drake_cc_googletest(
    name = "traffic_light_test",
    srcs = ["test/traffic_light_test.cc"],
    deps = [
        ":traffic_light",
    ],
)

drake_cc_googletest(
    name = "traffic_light_aware_controller_test",
    srcs = ["test/traffic_light_aware_controller_test.cc"],
    deps = [
        ":traffic_light_aware_controller",
    ],
)

add_lint_tests()
