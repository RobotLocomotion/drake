# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/systems/framework"]
cc_library(
    name = "generated_vectors",
    srcs = [
        "gen/driving_command.cc",
        "gen/euler_floating_joint_state.cc",
        "gen/idm_planner_parameters.cc",
        "gen/idm_with_trajectory_agent_parameters.cc",
        "gen/idm_with_trajectory_agent_state.cc",
        "gen/simple_car_config.cc",
        "gen/simple_car_state.cc",
    ],
    hdrs = [
        "gen/driving_command.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_planner_parameters.cc",
        "gen/idm_with_trajectory_agent_parameters.h",
        "gen/idm_with_trajectory_agent_state.h",
        "gen/simple_car_config.h",
        "gen/simple_car_state.h",
    ],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "generated_translators",
    srcs = [
        "gen/driving_command_translator.cc",
        "gen/euler_floating_joint_state_translator.cc",
        "gen/idm_with_trajectory_agent_state_translator.cc",
        "gen/simple_car_config_translator.cc",
        "gen/simple_car_state_translator.cc",
    ],
    hdrs = [
        "gen/driving_command_translator.h",
        "gen/euler_floating_joint_state_translator.h",
        "gen/idm_with_trajectory_agent_state_translator.h",
        "gen/simple_car_config_translator.h",
        "gen/simple_car_state_translator.h",
    ],
    deps = DEPS + [
        ":generated_vectors",
        "//drake/systems/lcm:translator",
        "//drake/lcmtypes:automotive",
    ],
    linkstatic = 1)

DEPS = [":generated_vectors"]
cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "linear_car",
    srcs = ["linear_car.cc"],
    hdrs = ["linear_car.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "idm_planner",
    srcs = ["idm_planner.cc"],
    hdrs = ["idm_planner.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "idm_with_trajectory_agent",
    srcs = ["idm_with_trajectory_agent.cc"],
    hdrs = ["idm_with_trajectory_agent.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "simple_car",
    srcs = ["simple_car.cc"],
    hdrs = ["simple_car.h", "simple_car_to_euler_floating_joint.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "single_lane_ego_and_agent",
    srcs = ["single_lane_ego_and_agent.cc"],
    hdrs = ["single_lane_ego_and_agent.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    deps = DEPS + [":curve2"],
    linkstatic = 1)
