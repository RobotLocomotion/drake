# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "utility",
    srcs = [
        "generate_obj.cc",
        "generate_urdf.cc",
    ],
    hdrs = [
        "generate_obj.h",
        "generate_urdf.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_binary(
    name = "yaml_to_obj",
    srcs = ["yaml_to_obj.cc"],
    deps = [
        ":utility",
        "//drake/automotive/maliput/monolane",
        "//drake/common:text_logging_gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "generate_urdf_test",
    deps = [
        ":utility",
        "//drake/automotive/maliput/monolane",
        "//drake/thirdParty:spruce",
    ],
)

py_test(
    name = "yaml_to_obj_test",
    size = "small",
    srcs = ["test/yaml_to_obj_test.py"],
    args = ["drake/automotive/maliput/utility/yaml_to_obj"],
    data = [
        ":yaml_to_obj",
        "//drake/automotive/maliput/monolane:yamls",
    ],
)

cpplint()
