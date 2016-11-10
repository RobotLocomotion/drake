# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["@gtest//:main"]

cc_test(
    name = "curve2_test",
    srcs = ["curve2_test.cc"],
    deps = DEPS + ["//drake/automotive:curve2"],
    size = "small")

cc_test(
    name = "simple_car_test",
    srcs = ["simple_car_test.cc"],
    deps = DEPS + ["//drake/automotive:simple_car"],
    size = "small")

cc_test(
    name = "idm_with_trajectory_agent_test",
    srcs = ["idm_with_trajectory_agent_test.cc"],
    deps = DEPS + ["//drake/automotive:idm_with_trajectory_agent"],
    size = "small")

cc_test(
    name = "simple_car_to_euler_floating_joint_test",
    srcs = ["simple_car_to_euler_floating_joint_test.cc"],
    deps = DEPS + ["//drake/automotive:simple_car"],
    size = "small")

cc_test(
    name = "trajectory_car_test",
    srcs = ["trajectory_car_test.cc"],
    deps = DEPS + ["//drake/automotive:trajectory_car"],
    size = "small")

cc_test(
    name = "simple_car_state_translator_test",
    srcs = ["simple_car_state_translator_test.cc"],
    deps = DEPS + ["//drake/automotive:generated_translators"],
    size = "small")
