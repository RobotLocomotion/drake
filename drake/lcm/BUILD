# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "interface",
    hdrs = [
        "drake_lcm_interface.h",
        "drake_lcm_message_handler_interface.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "mock",
    testonly = 1,
    srcs = ["drake_mock_lcm.cc"],
    hdrs = ["drake_mock_lcm.h"],
    deps = [
        ":interface",
    ],
)

drake_cc_library(
    name = "lcm",
    srcs = [
        "drake_lcm.cc",
        "lcm_receive_thread.cc",
    ],
    hdrs = [
        "drake_lcm.h",
        "lcm_receive_thread.h",
    ],
    deps = [
        ":interface",
        "//drake/common",
        "@lcm",
    ],
)

drake_cc_library(
    name = "lcmt_drake_signal_utils",
    testonly = 1,
    srcs = ["lcmt_drake_signal_utils.cc"],
    hdrs = ["lcmt_drake_signal_utils.h"],
    deps = [
        "//drake/common",
        "//drake/lcmtypes:drake_signal",
    ],
)

drake_cc_library(
    name = "translator_base",
    hdrs = [
        "translator_base.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common:unused",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "drake_lcm_test",
    local = 1,
    deps = [
        ":lcm",
        ":lcmt_drake_signal_utils",
    ],
)

drake_cc_googletest(
    name = "drake_mock_lcm_test",
    deps = [
        ":lcmt_drake_signal_utils",
        ":mock",
    ],
)

drake_cc_googletest(
    name = "lcmt_drake_signal_utils_test",
    deps = [
        ":lcmt_drake_signal_utils",
    ],
)

cpplint()
