# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/systems/framework"]
cc_library(
    name = "translator",
    srcs = [
        "lcm_and_vector_base_translator.cc",
        "lcm_translator_dictionary.cc",
    ],
    hdrs = [
        "lcm_and_vector_base_translator.h",
        "lcm_translator_dictionary.h"
    ],
    deps = DEPS,
    linkstatic=1)

cc_library(
    name = "lcm",
    srcs = [
        "lcm_publisher_system.cc",
        "lcm_subscriber_system.cc",
        "serializer.cc",
    ],
    hdrs = [
        "lcm_publisher_system.h",
        "lcm_subscriber_system.h",
        "serializer.h",
    ],
    deps = DEPS + [":translator", "//drake/lcm:interface"],
    linkstatic = 1)

cc_library(
    name = "lcmt_drake_signal_translator",
    srcs = ["lcmt_drake_signal_translator.cc"],
    hdrs = ["lcmt_drake_signal_translator.h"],
    deps = DEPS + [":translator", "//drake/lcmtypes:drake_signal"],
    linkstatic = 1)
