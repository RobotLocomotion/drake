# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:common"]

cc_library(
    name = "interface",
    hdrs = [
        "drake_lcm_interface.h",
        "drake_lcm_message_handler_interface.h",
    ],
    linkstatic = 1,
    deps = DEPS,
)

cc_library(
    name = "mock",
    testonly = 1,
    srcs = ["drake_mock_lcm.cc"],
    hdrs = ["drake_mock_lcm.h"],
    linkstatic = 1,
    deps = DEPS + [":interface"],
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
    deps = DEPS + [
        ":interface",
        "@lcm//:lcm",
    ],
)

cc_library(
    name = "lcmt_drake_signal_utils",
    testonly = 1,
    srcs = ["lcmt_drake_signal_utils.cc"],
    hdrs = ["lcmt_drake_signal_utils.h"],
    linkstatic = 1,
    deps = DEPS + ["//drake/lcmtypes:drake_signal"],
)
