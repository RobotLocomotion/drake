# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "gravity_compensator_test",
    size = "small",
    srcs = ["gravity_compensator_test.cc"],
    data = [
        "//drake/examples/SimpleFourBar:models",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/controllers:gravity_compensator",
        "@gtest//:main",
    ],
)

cc_test(
    name = "linear_optimal_control_test",
    size = "small",
    srcs = ["linear_optimal_control_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/controllers:linear_optimal_control",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pid_controlled_system_test",
    size = "small",
    srcs = ["pid_controlled_system_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/controllers:pid_controlled_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["pid_controller_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/controllers:pid_controller",
        "@gtest//:main",
    ],
)
