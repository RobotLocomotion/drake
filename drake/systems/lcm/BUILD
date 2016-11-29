# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

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
        "lcm_translator_dictionary.h",
    ],
    linkstatic = 1,
    deps = DEPS,
)

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
    linkstatic = 1,
    deps = DEPS + [
        ":translator",
        "//drake/lcm:interface",
    ],
)

cc_library(
    name = "lcmt_drake_signal_translator",
    srcs = ["lcmt_drake_signal_translator.cc"],
    hdrs = ["lcmt_drake_signal_translator.h"],
    linkstatic = 1,
    deps = DEPS + [
        ":translator",
        "//drake/lcmtypes:drake_signal",
    ],
)
