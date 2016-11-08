# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/lcm:mock",
    "//drake/lcm:lcmt_drake_signal_utils",
    "//drake/systems/lcm:lcmt_drake_signal_translator",
    "@gtest//:main"]

cc_test(
    name = "lcm_publisher_system_test",
    srcs = ["lcm_publisher_system_test.cc"],
    size = "small", deps = DEPS + ["//drake/systems/lcm:lcm"])

cc_test(
    name = "lcm_subscriber_system_test",
    srcs = ["lcm_subscriber_system_test.cc"],
    size = "small", deps = DEPS + ["//drake/systems/lcm:lcm"])

cc_test(
    name = "lcm_translator_dictionary_test",
    srcs = ["lcm_translator_dictionary_test.cc"],
    size = "small", deps = DEPS + ["//drake/systems/lcm:translator"])

cc_test(
    name = "serializer_test",
    srcs = ["serializer_test.cc"],
    size = "small", deps = DEPS + ["//drake/systems/lcm:lcm"])
