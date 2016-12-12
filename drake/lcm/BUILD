# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "drake_lcm_interface.h",
        "drake_lcm_message_handler_interface.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "mock",
    testonly = 1,
    srcs = ["drake_mock_lcm.cc"],
    hdrs = ["drake_mock_lcm.h"],
    linkstatic = 1,
    deps = [
        ":interface",
    ],
)

cc_library(
    name = "lcm",
    srcs = [
        "drake_lcm.cc",
        "lcm_receive_thread.cc",
    ],
    hdrs = [
        "drake_lcm.h",
        "lcm_receive_thread.h",
    ],
    linkstatic = 1,
    deps = [
        ":interface",
        "//drake/common",
        "@lcm//:lcm",
    ],
)

cc_component(
    name = "lcmt_drake_signal_utils",
    testonly = 1,
    deps = [
        "//drake/common",
        "//drake/lcmtypes:drake_signal",
    ],
)

cc_component(
    name = "lcm_call_matlab",
    testonly = 1,
    deps = [
        "//drake/common",
        "//drake/lcmtypes:call_matlab",
    ],
)

# === test/ ===

cc_unit_test(
    name = "drake_lcm_test",
    local = 1,
    deps = [
        ":lcm",
        ":lcmt_drake_signal_utils",
    ],
)

cc_unit_test(
    name = "drake_mock_lcm_test",
    deps = [
        ":lcmt_drake_signal_utils",
        ":mock",
    ],
)

cc_unit_test(
    name = "lcmt_drake_signal_utils_test",
    deps = [
        ":lcmt_drake_signal_utils",
    ],
)

cc_unit_test(
    name = "lcm_call_matlab_test",
    local = 1,
    deps = [
        ":lcm_call_matlab",
        ":lcmt_drake_signal_utils",
    ],
)
