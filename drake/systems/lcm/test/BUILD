# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/lcm:mock",
    "//drake/lcm:lcmt_drake_signal_utils",
    "//drake/systems/lcm:lcmt_drake_signal_translator",
    "@gtest//:main",
]

cc_test(
    name = "lcm_publisher_system_test",
    size = "small",
    srcs = ["lcm_publisher_system_test.cc"],
    deps = DEPS + ["//drake/systems/lcm:lcm"],
)

cc_test(
    name = "lcm_subscriber_system_test",
    size = "small",
    srcs = ["lcm_subscriber_system_test.cc"],
    deps = DEPS + ["//drake/systems/lcm:lcm"],
)

cc_test(
    name = "lcm_translator_dictionary_test",
    size = "small",
    srcs = ["lcm_translator_dictionary_test.cc"],
    deps = DEPS + ["//drake/systems/lcm:translator"],
)

cc_test(
    name = "serializer_test",
    size = "small",
    srcs = ["serializer_test.cc"],
    deps = DEPS + ["//drake/systems/lcm:lcm"],
)
