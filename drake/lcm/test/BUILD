# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/lcm:lcmt_drake_signal_utils",
    "@gtest//:main",
]

cc_test(
    name = "drake_lcm_test",
    size = "small",
    srcs = ["drake_lcm_test.cc"],
    local = 1,
    deps = DEPS + ["//drake/lcm:lcm"],
)

cc_test(
    name = "drake_mock_lcm_test",
    size = "small",
    srcs = ["drake_mock_lcm_test.cc"],
    deps = DEPS + ["//drake/lcm:mock"],
)

cc_test(
    name = "lcmt_drake_signal_utils_test",
    size = "small",
    srcs = ["lcmt_drake_signal_utils_test.cc"],
    deps = DEPS,
)

cc_test(
    name = "lcm_call_matlab_test",
    size = "small",
    srcs = ["lcm_call_matlab_test.cc"],
    deps = DEPS + ["//drake/lcm:lcm_call_matlab"],
)
