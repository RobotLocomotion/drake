# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

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
    deps = [
        "//drake/systems/framework",
    ],
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
    deps = [
        ":generated_vectors",
        "//drake/lcmtypes:automotive",
        "//drake/systems/framework",
        "//drake/systems/lcm:translator",
    ],
)

cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
    ],
)

cc_library(
    name = "linear_car",
    srcs = ["linear_car.cc"],
    hdrs = ["linear_car.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

cc_library(
    name = "idm_planner",
    srcs = ["idm_planner.cc"],
    hdrs = ["idm_planner.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

cc_library(
    name = "simple_car",
    srcs = ["simple_car.cc"],
    hdrs = [
        "simple_car.h",
        "simple_car_to_euler_floating_joint.h",
    ],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

cc_library(
    name = "single_lane_ego_and_agent",
    srcs = ["single_lane_ego_and_agent.cc"],
    hdrs = ["single_lane_ego_and_agent.h"],
    linkstatic = 1,
    deps = [
        ":idm_planner",
        ":linear_car",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    linkstatic = 1,
    deps = [
        ":curve2",
        ":generated_vectors",
    ],
)

cc_library(
    name = "automotive_simulator",
    srcs = ["automotive_simulator.cc"],
    hdrs = ["automotive_simulator.h"],
    linkstatic = 1,
    deps = [
        ":generated_translators",
        ":generated_vectors",
        ":simple_car",
        ":trajectory_car",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:multiplexer",
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
    srcs = [
        "car_sim_lcm.cc",
        "car_sim_lcm_common.cc",
        "car_sim_lcm_common.h",
    ],
    linkstatic = 1,
    deps = [
        ":automotive_simulator",
        "//drake/common:text_logging_gflags",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/controllers:pid_controlled_system",
    ],
)

py_library(
    name = "drake_paths",
    srcs = ["drake_paths.py"],
)

py_binary(
    name = "steering_command_driver",
    srcs = ["steering_command_driver.py"],
    deps = [
        ":drake_paths",
        "//drake/lcmtypes:py_automotive",
        "@lcm//:lcm-python",
    ],
)

filegroup(
    name = "models",
    srcs = glob(["models/**"]),
)

# === test/ ===

cc_googletest(
    name = "automotive_simulator_test",
    data = ["//drake/automotive:models"],
    local = 1,
    deps = [
        "//drake/automotive:automotive_simulator",
        "//drake/lcm:mock",
    ],
)

cc_googletest(
    name = "curve2_test",
    deps = [
        "//drake/automotive:curve2",
    ],
)

cc_googletest(
    name = "simple_car_test",
    deps = [
        "//drake/automotive:simple_car",
    ],
)

cc_googletest(
    name = "idm_planner_test",
    deps = [
        "//drake/automotive:idm_planner",
    ],
)

cc_googletest(
    name = "linear_car_test",
    deps = [
        "//drake/automotive:linear_car",
    ],
)

cc_googletest(
    name = "simple_car_to_euler_floating_joint_test",
    deps = [
        "//drake/automotive:simple_car",
    ],
)

cc_googletest(
    name = "single_lane_ego_and_agent_test",
    deps = [
        "//drake/automotive:single_lane_ego_and_agent",
    ],
)

cc_googletest(
    name = "trajectory_car_test",
    deps = [
        "//drake/automotive:trajectory_car",
    ],
)

cc_googletest(
    name = "simple_car_state_translator_test",
    deps = [
        "//drake/automotive:generated_translators",
    ],
)
