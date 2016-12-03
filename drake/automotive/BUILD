# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/systems/framework"]

cc_library(
    name = "generated_vectors",
    srcs = [
        "gen/driving_command.cc",
        "gen/euler_floating_joint_state.cc",
        "gen/idm_planner_parameters.cc",
        "gen/simple_car_config.cc",
        "gen/simple_car_state.cc",
    ],
    hdrs = [
        "gen/driving_command.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_planner_parameters.h",
        "gen/simple_car_config.h",
        "gen/simple_car_state.h",
    ],
    linkstatic = 1,
    deps = DEPS,
)

cc_library(
    name = "generated_translators",
    srcs = [
        "gen/driving_command_translator.cc",
        "gen/euler_floating_joint_state_translator.cc",
        "gen/simple_car_config_translator.cc",
        "gen/simple_car_state_translator.cc",
    ],
    hdrs = [
        "gen/driving_command_translator.h",
        "gen/euler_floating_joint_state_translator.h",
        "gen/simple_car_config_translator.h",
        "gen/simple_car_state_translator.h",
    ],
    linkstatic = 1,
    deps = DEPS + [
        ":generated_vectors",
        "//drake/systems/lcm:translator",
        "//drake/lcmtypes:automotive",
    ],
)

DEPS = [":generated_vectors"]

cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    linkstatic = 1,
    deps = DEPS,
)

cc_library(
    name = "linear_car",
    srcs = ["linear_car.cc"],
    hdrs = ["linear_car.h"],
    linkstatic = 1,
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_library(
    name = "idm_planner",
    srcs = ["idm_planner.cc"],
    hdrs = ["idm_planner.h"],
    linkstatic = 1,
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_library(
    name = "simple_car",
    srcs = ["simple_car.cc"],
    hdrs = [
        "simple_car.h",
        "simple_car_to_euler_floating_joint.h",
    ],
    linkstatic = 1,
    deps = DEPS + ["//drake/common:symbolic"],
)

cc_library(
    name = "single_lane_ego_and_agent",
    srcs = ["single_lane_ego_and_agent.cc"],
    hdrs = ["single_lane_ego_and_agent.h"],
    linkstatic = 1,
    deps = DEPS + [
        "//drake/systems/framework/primitives:constant_vector_source",
        ":linear_car",
        ":idm_planner",
    ],
)

cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    linkstatic = 1,
    deps = DEPS + [":curve2"],
)

cc_library(
    name = "automotive_simulator",
    srcs = ["automotive_simulator.cc"],
    hdrs = ["automotive_simulator.h"],
    linkstatic = 1,
    deps = DEPS + [
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:multiplexer",
        ":generated_translators",
        ":simple_car",
        ":trajectory_car",
    ],
)

cc_binary(
    name = "automotive_demo",
    srcs = [
        "automotive_demo.cc",
        "create_trajectory_params.cc",
        "create_trajectory_params.h",
    ],
    linkstatic = 1,
    deps = [
        ":automotive_simulator",
        "//drake/common:text_logging_gflags",
    ],
)

cc_binary(
    name = "car_sim_lcm",
    srcs = ["car_sim_lcm.cc"],
    linkstatic = 1,
    deps = [
        ":automotive_simulator",
        "//drake/common:text_logging_gflags",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/controllers:pid_controlled_system",
    ],
)

filegroup(
    name = "models",
    srcs = glob(["models/**"]),
)
