# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "point_t_translator",
    testonly = 1,
    srcs = ["point_t_translator.cc"],
    hdrs = ["point_t_translator.h"],
    deps = [
        "//drake/lcm:translator_base",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_library(
    name = "pose_t_translator",
    testonly = 1,
    srcs = ["pose_t_translator.cc"],
    hdrs = ["pose_t_translator.h"],
    deps = [
        ":point_t_translator",
        ":quaternion_t_translator",
        "//drake/lcm:translator_base",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_library(
    name = "quaternion_t_translator",
    testonly = 1,
    srcs = ["quaternion_t_translator.cc"],
    hdrs = ["quaternion_t_translator.h"],
    deps = [
        "//drake/lcm:translator_base",
        "@robotlocomotion_lcmtypes",
    ],
)

