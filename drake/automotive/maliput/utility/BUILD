# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utility",
    srcs = [
        "generate_obj.cc",
        "generate_urdf.cc",
        "infinite_circuit_road.cc"
    ],
    hdrs = [
        "generate_obj.h",
        "generate_urdf.h",
        "infinite_circuit_road.h"
    ],
    linkstatic = 1,
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

cc_binary(
    name = "yaml_to_obj",
    srcs = ["yaml_to_obj.cc"],
    deps = [
        ":utility",
        "//drake/automotive/maliput/monolane",
        "//drake/common:text_logging_gflags",
    ],
)


# === test/ ===

cc_test(
    name = "generate_obj_test",
    size = "small",
    srcs = ["test/generate_obj_test.cc"],
    deps = [
        ":utility",
        "//drake/automotive/maliput/monolane",
        "@gtest//:main",
    ],
)

# TODO(maddog@tri.global)  Run this test somehow, too.
#py_test(
#    name = "yaml_to_obj_test",
#    srcs = ["test/yaml_to_obj_test.py"],
#)
