# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:__subpackages__"],
)

drake_cc_library(
    name = "kuka_pick_and_place_monolithic",
    srcs = [
        "demo_diagram_builder.cc",
        "state_machine_system.cc",
    ],
    hdrs = [
        "demo_diagram_builder.h",
        "pick_and_place_common.h",
        "state_machine_system.h",
    ],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives",
        "//drake/examples/kuka_iiwa_arm/dev/pick_and_place:kuka_pick_and_place",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "monolithic_pick_and_place_demo",
    srcs = ["monolithic_pick_and_place_demo.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":kuka_pick_and_place_monolithic",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "trajectory_generator_test",
    srcs = ["trajectory_generator_test.cc"],
    deps = [
        ":kuka_pick_and_place_monolithic",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/lcm",
        "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
