# -*- python -*

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])
DEPS = ["//drake/common:common"]

cc_library(
    name = "interface",
    hdrs = [
        "drake_lcm_interface.h",
        "drake_lcm_message_handler_interface.h",
    ],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "mock",
    srcs = ["drake_mock_lcm.cc"],
    hdrs = ["drake_mock_lcm.h"],
    deps = DEPS + [":interface"],
    linkstatic = 1,
    testonly = 1)

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
    deps = DEPS + [":interface", "@lcm//:lcm"],
    linkstatic = 1)

cc_library(
    name = "lcmt_drake_signal_utils",
    srcs = ["lcmt_drake_signal_utils.cc"],
    hdrs = ["lcmt_drake_signal_utils.h"],
    deps = DEPS + ["//drake/lcmtypes:drake_signal"],
    linkstatic = 1,
    testonly = 1)
