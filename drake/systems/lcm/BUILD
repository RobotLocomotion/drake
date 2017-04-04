# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "translator",
    srcs = [
        "lcm_and_vector_base_translator.cc",
        "lcm_translator_dictionary.cc",
    ],
    hdrs = [
        "lcm_and_vector_base_translator.h",
        "lcm_translator_dictionary.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
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
    deps = [
        ":translator",
        "//drake/lcm:interface",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "lcmt_drake_signal_translator",
    srcs = ["lcmt_drake_signal_translator.cc"],
    hdrs = ["lcmt_drake_signal_translator.h"],
    deps = [
        ":translator",
        "//drake/lcmtypes:drake_signal",
        "//drake/systems/framework",
    ],
)

# === test ===

drake_cc_googletest(
    name = "lcm_publisher_system_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
        "//drake/systems/analysis",
    ],
)

drake_cc_googletest(
    name = "lcm_subscriber_system_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
        "//drake/systems/framework/test_utilities:my_vector",
    ],
)

drake_cc_googletest(
    name = "lcm_translator_dictionary_test",
    deps = [
        ":lcmt_drake_signal_translator",
        ":translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)

drake_cc_googletest(
    name = "serializer_test",
    deps = [
        ":lcm",
        ":lcmt_drake_signal_translator",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
    ],
)

cpplint()
