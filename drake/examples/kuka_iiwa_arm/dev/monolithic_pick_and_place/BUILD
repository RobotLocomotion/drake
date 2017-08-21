# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")
load(":pick_and_place_test.bzl", "pick_and_place_test")

drake_cc_library(
    name = "kuka_pick_and_place_monolithic",
    srcs = [
        "state_machine_system.cc",
    ],
    hdrs = [
        "state_machine_system.h",
    ],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
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
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/manipulation/planner:robot_plan_interpolator",
        "//drake/manipulation/schunk_wsg:schunk_wsg_controller",
        "//drake/manipulation/util:world_sim_tree_builder",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/systems/analysis:simulator",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_bot2_core",
    ],
)

pick_and_place_test(
    name = "pick_and_place_scenario_1",
    # TODO(sam.creasey) This should be target 1, but that requries
    # offsetting the gripper location to deal with the larger box,
    # which isn't currently supported.
    args = [
        "--target=2",
        "--orientation=1.57",
        "--start-position=0",
        "--end-position=2",
        # TODO(sam.creasey) This is an artisanally crafted timestep
        # intended to dodge sensitive dependence on initial conditions
        # in the IK planner.  Depending on slight changes in the arm's
        # configration after lifting the item from the pick, the IK
        # planner (particularly when running under SNOPT) can produce
        # radically different results.  This value produces usable
        # (but different!) results on Mac vs. Linux builds.  See #6811
        # for more information.
        "--dt 6e-4",
    ],
    # TODO(sam.creasey) IPOPT doesn't find a reasonable solution for
    # one of the steps in this demo.  We should see if this improves
    # when #3128 is fixed.
    tags = ["snopt"],
)

pick_and_place_test(
    name = "pick_and_place_scenario_2",
    args = [
        "--target=2",
        "--orientation=-1.57",
        "--start-position=4",
        "--end-position=2",
        # The smaller timestep is necessary to keep the box from
        # sliding out of the target region when bumped by the
        # gripper after being placed.
        "--dt 5e-4",
    ],
    # TODO(sam.creasey) IPOPT doesn't find a reasonable solution for
    # one of the steps in this demo.  We should see if this improves
    # when #3128 is fixed.
    tags = ["snopt"],
)

pick_and_place_test(
    name = "pick_and_place_scenario_3",
    args = [
        "--target=2",
        "--orientation=-1.57",
        "--start-position=4",
        "--end-position=1",
        # The smaller timestep is necessary to keep
        # the box from bouncing out of the target
        # region when placed.
        "--dt 5e-4",
    ],
    # TODO(sam.creasey) IPOPT doesn't find a reasonable solution for
    # one of the steps in this demo.  We should see if this improves
    # when #3128 is fixed.
    tags = ["snopt"],
)

pick_and_place_test(
    name = "pick_and_place_scenario_4",
    # TODO(sam.creasey) This should be a smaller rectangular target.
    args = [
        "--target=2",
        "--start-position=1",
        "--end-position=3",
        # The smaller timestep is necessary to keep
        # the box from bouncing out of the target
        # region when placed.
        "--dt 5e-4",
    ],
)

pick_and_place_test(
    name = "pick_and_place_scenario_5",
    # TODO(sam.creasey) This should be a smaller cylindrical target,
    # but we don't currently have a cylinder with good collision
    # geometry.
    args = [
        "--target=0",
        "--start-position=2",
        "--end-position=0",
        # TODO(sam.creasey) This is an artisanally crafted
        # timestep intended to dodge sensitive dependence on
        # initial conditions in the IK planner.  If the block
        # slips in the gripper in just the wrong way (which only
        # happens on some dt values) then the arm tries to move
        # into a radically different configuration in the process
        # of placing the block at the target.
        "--dt 6e-4",
    ],
)

add_lint_tests()
