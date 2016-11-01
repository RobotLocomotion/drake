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
        "gen/idm_with_trajectory_agent_parameters.cc",
        "gen/idm_with_trajectory_agent_state.cc",
        "gen/simple_car_config.cc",
        "gen/simple_car_state.cc",
    ],
    hdrs = [
        "gen/driving_command.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_with_trajectory_agent_parameters.h",
        "gen/idm_with_trajectory_agent_state.h",
        "gen/simple_car_config.h",
        "gen/simple_car_state.h",
    ],
    deps = DEPS,
    linkstatic = 1)

DEPS = [":generated_vectors"]
cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    deps = DEPS,
    linkstatic = 1)
cc_library(
    name = "idm_with_trajectory_agent",
    srcs = ["idm_with_trajectory_agent.cc"],
    hdrs = ["idm_with_trajectory_agent.h", "idm_with_trajectory_agent-inl.h"],
    deps = DEPS,
    linkstatic = 1)
cc_library(
    name = "simple_car",
    srcs = ["simple_car.cc"],
    hdrs = ["simple_car.h", "simple_car-inl.h"],
    deps = DEPS,
    linkstatic = 1)
cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    deps = DEPS + [":curve2"],
    linkstatic = 1)
