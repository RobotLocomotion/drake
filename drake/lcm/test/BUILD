# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/lcm:lcmt_drake_signal_utils", "@gtest//:main"]

cc_test(
    name = "drake_lcm_test",
    srcs = ["drake_lcm_test.cc"],
    size = "small", deps = DEPS + ["//drake/lcm:lcm"],
    local = 1)

cc_test(
    name = "drake_mock_lcm_test",
    srcs = ["drake_mock_lcm_test.cc"],
    size = "small", deps = DEPS + ["//drake/lcm:mock"])

cc_test(
    name = "lcmt_drake_signal_utils_test",
    srcs = ["lcmt_drake_signal_utils_test.cc"],
    size = "small", deps = DEPS)
