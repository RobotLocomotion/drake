# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = ["@gtest//:main"]

cc_test(
    name = "automotive_simulator_test",
    size = "small",
    srcs = ["automotive_simulator_test.cc"],
    data = ["//drake/automotive:models"],
    deps = DEPS + [
        "//drake/automotive:automotive_simulator",
        "//drake/lcm:mock",
    ],
)

cc_test(
    name = "curve2_test",
    size = "small",
    srcs = ["curve2_test.cc"],
    deps = DEPS + ["//drake/automotive:curve2"],
)

cc_test(
    name = "simple_car_test",
    size = "small",
    srcs = ["simple_car_test.cc"],
    deps = DEPS + ["//drake/automotive:simple_car"],
)

cc_test(
    name = "idm_planner_test",
    size = "small",
    srcs = ["idm_planner_test.cc"],
    deps = DEPS + ["//drake/automotive:idm_planner"],
)

cc_test(
    name = "linear_car_test",
    size = "small",
    srcs = ["linear_car_test.cc"],
    deps = DEPS + ["//drake/automotive:linear_car"],
)

cc_test(
    name = "simple_car_to_euler_floating_joint_test",
    size = "small",
    srcs = ["simple_car_to_euler_floating_joint_test.cc"],
    deps = DEPS + ["//drake/automotive:simple_car"],
)

cc_test(
    name = "single_lane_ego_and_agent_test",
    size = "small",
    srcs = ["single_lane_ego_and_agent_test.cc"],
    deps = DEPS + ["//drake/automotive:single_lane_ego_and_agent"],
)

cc_test(
    name = "trajectory_car_test",
    size = "small",
    srcs = ["trajectory_car_test.cc"],
    deps = DEPS + ["//drake/automotive:trajectory_car"],
)

cc_test(
    name = "simple_car_state_translator_test",
    size = "small",
    srcs = ["simple_car_state_translator_test.cc"],
    deps = DEPS + ["//drake/automotive:generated_translators"],
)
