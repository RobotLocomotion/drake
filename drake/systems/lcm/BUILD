# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

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
    deps = [
        "//drake/systems/framework",
    ],
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
    deps = [
        ":translator",
        "//drake/lcm:interface",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "lcmt_drake_signal_translator",
    srcs = ["lcmt_drake_signal_translator.cc"],
    hdrs = ["lcmt_drake_signal_translator.h"],
    linkstatic = 1,
    deps = [
        ":translator",
        "//drake/lcmtypes:drake_signal",
        "//drake/systems/framework",
    ],
)

# === test ===

cc_googletest(
    name = "lcm_publisher_system_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)

cc_googletest(
    name = "lcm_subscriber_system_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)

cc_googletest(
    name = "lcm_translator_dictionary_test",
    deps = [
        ":lcmt_drake_signal_translator",
        ":translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)

cc_googletest(
    name = "serializer_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)
