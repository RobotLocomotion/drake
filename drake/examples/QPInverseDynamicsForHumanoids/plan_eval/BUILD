# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = [
        "//drake/examples/QPInverseDynamicsForHumanoids:__subpackages__",
        "//drake/examples/kuka_iiwa_arm:__subpackages__",
    ],
)

drake_cc_library(
    name = "plan_eval_utils",
    srcs = [
        "plan_eval_utils.cc",
    ],
    hdrs = [
        "plan_eval_utils.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/multibody:rigid_body_tree",
    ],
)

# === test/ ===
drake_cc_googletest(
    name = "plan_eval_utils_test",
    deps = [
        ":plan_eval_utils",
        "//drake/common:eigen_matrix_compare",
    ],
)
