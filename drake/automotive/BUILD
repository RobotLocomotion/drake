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

drake_cc_library(
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

drake_cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
    ],
)

drake_cc_library(
    name = "linear_car",
    srcs = ["linear_car.cc"],
    hdrs = ["linear_car.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "idm_planner",
    srcs = ["idm_planner.cc"],
    hdrs = ["idm_planner.h"],
    linkstatic = 1,
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
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

drake_cc_library(
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

drake_cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    linkstatic = 1,
    deps = [
        ":curve2",
        ":generated_vectors",
    ],
)

drake_cc_library(
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

drake_cc_binary(
    name = "automotive_demo",
    srcs = [
        "automotive_demo.cc",
        "create_trajectory_params.cc",
        "create_trajectory_params.h",
    ],
    data = [
        ":models",
    ],
    deps = [
        ":automotive_simulator",
        "//drake/common:text_logging_gflags",
    ],
)

drake_cc_binary(
    name = "car_sim_lcm",
    srcs = [
        "car_sim_lcm.cc",
        "car_sim_lcm_common.cc",
        "car_sim_lcm_common.h",
    ],
    data = [
        ":models",
    ],
    deps = [
        ":automotive_simulator",
        "//drake/common:text_logging_gflags",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/controllers:pid_controlled_system",
    ],
)

py_binary(
    name = "steering_command_driver",
    srcs = ["steering_command_driver.py"],
    deps = [
        "//drake/lcmtypes:automotive-py",
        "@lcm//:lcm-python",
    ],
)

java_binary(
    name = "lcm-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [
        "//drake/lcmtypes:automotive-java",
        "//drake/lcmtypes:viewer-java",
    ],
)

py_binary(
    name = "demo",
    srcs = ["automotive_demo.py"],
    data = [
        ":automotive_demo",
        ":lcm-spy",
        ":steering_command_driver",
        "@drake_visualizer//:drake-visualizer",
        "@lcm//:lcm-logger",
    ],
    main = "automotive_demo.py",
    deps = [
        "@lcm//:lcm-python",
    ],
)

genrule(
    name = "speed_bump_genrule",
    srcs = ["models/speed_bump/speed_bump.yaml"],
    outs = ["models/speed_bump/speed_bump.obj"],
    cmd = " ".join([
        "$(location //drake/automotive/maliput/utility:yaml_to_obj)",
        "--yaml_file '$<'",
        "--obj_dir `dirname '$@'`",
        "--obj_file `basename '$@' .obj`",
    ]),
    tools = ["//drake/automotive/maliput/utility:yaml_to_obj"],
)

filegroup(
    name = "models",
    srcs = glob(["models/**"]) + [
        "models/speed_bump/speed_bump.obj",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "automotive_simulator_test",
    data = ["//drake/automotive:models"],
    local = 1,
    deps = [
        "//drake/automotive:automotive_simulator",
        "//drake/lcm:mock",
    ],
)

drake_cc_googletest(
    name = "curve2_test",
    deps = [
        "//drake/automotive:curve2",
    ],
)

drake_cc_googletest(
    name = "simple_car_test",
    deps = [
        "//drake/automotive:simple_car",
    ],
)

drake_cc_googletest(
    name = "idm_planner_test",
    deps = [
        "//drake/automotive:idm_planner",
    ],
)

drake_cc_googletest(
    name = "linear_car_test",
    deps = [
        "//drake/automotive:linear_car",
    ],
)

drake_cc_googletest(
    name = "simple_car_to_euler_floating_joint_test",
    deps = [
        "//drake/automotive:simple_car",
    ],
)

drake_cc_googletest(
    name = "single_lane_ego_and_agent_test",
    deps = [
        "//drake/automotive:single_lane_ego_and_agent",
    ],
)

drake_cc_googletest(
    name = "trajectory_car_test",
    deps = [
        "//drake/automotive:trajectory_car",
    ],
)

drake_cc_googletest(
    name = "simple_car_state_translator_test",
    deps = [
        "//drake/automotive:generated_translators",
    ],
)

cpplint()
