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
    name = "idm_planner_test",
    srcs = ["idm_planner_test.cc"],
    deps = DEPS + ["//drake/automotive:idm_planner"],
    size = "small")

cc_test(
    name = "linear_car_test",
    srcs = ["linear_car_test.cc"],
    deps = DEPS + ["//drake/automotive:linear_car"],
    size = "small")

cc_test(
    name = "simple_car_to_euler_floating_joint_test",
    srcs = ["simple_car_to_euler_floating_joint_test.cc"],
    deps = DEPS + ["//drake/automotive:simple_car"],
    size = "small")

cc_test(
    name = "single_lane_ego_and_agent_test",
    srcs = ["single_lane_ego_and_agent_test.cc"],
    deps = DEPS + ["//drake/automotive:single_lane_ego_and_agent"],
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
