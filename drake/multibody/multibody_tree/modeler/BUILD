# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "multibody_modeler",
    srcs = [
        "multibody_modeler.cc",
    ],
    hdrs = [
        "modeler_ids.h",
        "multibody_modeler.h",
    ],
    deps = [
        "//drake/geometry:identifier",
        "//drake/multibody/multibody_tree",
    ],
)

# === test/ ===

#drake_cc_googletest(
#    name = "double_pendulum_test",
#    deps = [
#        ":multibody_tree",
#        "//drake/multibody/benchmarks/acrobot",
#    ],
#)

add_lint_tests()
