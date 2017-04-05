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
        "gen/bicycle_car_parameters.cc",
        "gen/bicycle_car_state.cc",
        "gen/driving_command.cc",
        "gen/euler_floating_joint_state.cc",
        "gen/idm_planner_parameters.cc",
        "gen/maliput_railcar_params.cc",
        "gen/maliput_railcar_state.cc",
        "gen/pure_pursuit_params.cc",
        "gen/simple_car_params.cc",
        "gen/simple_car_state.cc",
    ],
    hdrs = [
        "gen/bicycle_car_parameters.h",
        "gen/bicycle_car_state.h",
        "gen/driving_command.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_planner_parameters.h",
        "gen/maliput_railcar_params.h",
        "gen/maliput_railcar_state.h",
        "gen/pure_pursuit_params.h",
        "gen/simple_car_params.h",
        "gen/simple_car_state.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "generated_translators",
    srcs = [
        "gen/bicycle_car_parameters_translator.cc",
        "gen/bicycle_car_state_translator.cc",
        "gen/driving_command_translator.cc",
        "gen/euler_floating_joint_state_translator.cc",
        "gen/maliput_railcar_params_translator.cc",
        "gen/maliput_railcar_state_translator.cc",
        "gen/simple_car_params_translator.cc",
        "gen/simple_car_state_translator.cc",
    ],
    hdrs = [
        "gen/bicycle_car_parameters_translator.h",
        "gen/bicycle_car_state_translator.h",
        "gen/driving_command_translator.h",
        "gen/euler_floating_joint_state_translator.h",
        "gen/maliput_railcar_params_translator.h",
        "gen/maliput_railcar_state_translator.h",
        "gen/simple_car_params_translator.h",
        "gen/simple_car_state_translator.h",
    ],
    deps = [
        ":generated_vectors",
        "//drake/lcmtypes:automotive",
        "//drake/systems/framework",
        "//drake/systems/lcm:translator",
    ],
)

drake_cc_library(
    name = "bicycle_car",
    srcs = ["bicycle_car.cc"],
    hdrs = ["bicycle_car.h"],
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "box_car_vis",
    srcs = ["box_car_vis.cc"],
    hdrs = ["box_car_vis.h"],
    deps = [
        ":car_vis",
        "//drake/multibody/shapes",
        "//drake/systems/rendering:drake_visualizer_client",
    ],
)

drake_cc_library(
    name = "car_vis",
    srcs = ["car_vis.cc"],
    hdrs = ["car_vis.h"],
    deps = [
        "//drake/lcmtypes:viewer",
        "//drake/systems/rendering:pose_bundle",
    ],
)

drake_cc_library(
    name = "car_vis_applicator",
    srcs = ["car_vis_applicator.cc"],
    hdrs = ["car_vis_applicator.h"],
    deps = [
        ":box_car_vis",
        ":car_vis",
        ":generated_vectors",
        "//drake/lcmtypes:viewer",
        "//drake/systems/framework",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_aggregator",
    ],
)

drake_cc_library(
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    deps = [
        ":generated_vectors",
    ],
)

drake_cc_library(
    name = "idm_controller",
    srcs = ["idm_controller.cc"],
    hdrs = ["idm_controller.h"],
    deps = [
        ":generated_vectors",
        ":idm_planner",
        ":pose_selector",
        "//drake/automotive/maliput/api",
        "//drake/math:saturate",
        "//drake/systems/rendering:pose_bundle",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "idm_planner",
    srcs = ["idm_planner.cc"],
    hdrs = ["idm_planner.h"],
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "lane_direction",
    srcs = ["lane_direction.cc"],
    hdrs = ["lane_direction.h"],
    deps = [
        "//drake/automotive/maliput/api",
    ],
)

drake_cc_library(
    name = "maliput_railcar",
    srcs = ["maliput_railcar.cc"],
    hdrs = ["maliput_railcar.h"],
    deps = [
        ":calc_smooth_acceleration",
        ":generated_vectors",
        ":lane_direction",
        "//drake/automotive/maliput/api",
        "//drake/math:geometric_transform",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "monolane_onramp_merge",
    srcs = ["monolane_onramp_merge.cc"],
    hdrs = ["monolane_onramp_merge.h"],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/automotive/maliput/monolane",
    ],
)

drake_cc_library(
    name = "pose_selector",
    srcs = ["pose_selector.cc"],
    hdrs = ["pose_selector.h"],
    visibility = [],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/systems/rendering:pose_bundle",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "prius_vis",
    srcs = ["prius_vis.cc"],
    hdrs = ["prius_vis.h"],
    data = ["//drake/automotive:models"],
    deps = [
        ":car_vis",
        "//drake/common:drake_path",
        "//drake/lcmtypes:viewer",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:create_load_robot_message",
        "//drake/systems/rendering:pose_bundle",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "pure_pursuit",
    srcs = ["pure_pursuit.cc"],
    hdrs = ["pure_pursuit.h"],
    deps = [
        ":generated_vectors",
        ":lane_direction",
        ":pose_selector",
        "//drake/automotive/maliput/api",
        "//drake/common:symbolic",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "pure_pursuit_controller",
    srcs = ["pure_pursuit_controller.cc"],
    hdrs = ["pure_pursuit_controller.h"],
    deps = [
        ":generated_vectors",
        ":lane_direction",
        ":pose_selector",
        ":pure_pursuit",
        "//drake/automotive/maliput/api",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "road_path",
    srcs = ["road_path.cc"],
    hdrs = ["road_path.h"],
    visibility = [],
    deps = [
        ":lane_direction",
        ":monolane_onramp_merge",
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories/qp_spline:spline_generation",
        "//drake/math:saturate",
    ],
)

drake_cc_library(
    name = "simple_car",
    srcs = ["simple_car.cc"],
    hdrs = [
        "simple_car.h",
        "simple_car_to_euler_floating_joint.h",
    ],
    deps = [
        ":calc_smooth_acceleration",
        ":generated_vectors",
        "//drake/common:cond",
        "//drake/common:double",
        "//drake/common:symbolic",
        "//drake/math:saturate",
        "//drake/systems/rendering:frame_velocity",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "simple_powertrain",
    srcs = ["simple_powertrain.cc"],
    hdrs = ["simple_powertrain.h"],
    deps = [
        "//drake/systems/primitives:linear_system",
    ],
)

drake_cc_library(
    name = "calc_smooth_acceleration",
    srcs = ["calc_smooth_acceleration.cc"],
    hdrs = ["calc_smooth_acceleration.h"],
    visibility = [],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:symbolic",
    ],
)

drake_cc_library(
    name = "trajectory_car",
    srcs = ["trajectory_car.cc"],
    hdrs = ["trajectory_car.h"],
    deps = [
        ":curve2",
        ":generated_vectors",
        "//drake/systems/rendering:frame_velocity",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "automotive_simulator",
    srcs = ["automotive_simulator.cc"],
    hdrs = ["automotive_simulator.h"],
    deps = [
        ":car_vis_applicator",
        ":generated_translators",
        ":generated_vectors",
        ":maliput_railcar",
        ":prius_vis",
        ":simple_car",
        ":trajectory_car",
        "//drake/automotive/maliput/api",
        "//drake/automotive/maliput/utility",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/rendering:pose_aggregator",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
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
        ":monolane_onramp_merge",
        "//drake/automotive/maliput/dragway",
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
    add_test_rule = 1,
    data = [
        ":models",
    ],
    test_rule_args = [" --simulation_sec=0.01"],
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
        "//drake/lcmtypes:lcmtypes-py",
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
        "//drake/automotive/maliput/dragway",
        "//drake/lcm:mock",
    ],
)

drake_cc_googletest(
    name = "bicycle_car_test",
    deps = [
        "//drake/automotive:bicycle_car",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "box_car_vis_test",
    deps = [
        ":box_car_vis",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_googletest(
    name = "car_vis_applicator_test",
    deps = [
        ":car_vis_applicator",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_googletest(
    name = "curve2_test",
    deps = [
        "//drake/automotive:curve2",
    ],
)

drake_cc_googletest(
    name = "maliput_railcar_test",
    deps = [
        "//drake/automotive:maliput_railcar",
        "//drake/automotive/maliput/api",
        "//drake/automotive/maliput/dragway",
        "//drake/automotive/maliput/monolane",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_googletest(
    name = "monolane_onramp_merge_test",
    deps = [
        ":monolane_onramp_merge",
        "//drake/automotive/maliput/utility",
    ],
)

drake_cc_googletest(
    name = "simple_car_test",
    deps = [
        "//drake/automotive:simple_car",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "idm_controller_test",
    deps = [
        "//drake/automotive:idm_controller",
        "//drake/automotive/maliput/dragway",
    ],
)

drake_cc_googletest(
    name = "idm_planner_test",
    deps = [
        "//drake/automotive:idm_planner",
    ],
)

drake_cc_googletest(
    name = "pose_selector_test",
    deps = [
        "//drake/automotive:pose_selector",
        "//drake/automotive/maliput/dragway",
    ],
)

drake_cc_googletest(
    name = "prius_vis_test",
    deps = [
        ":prius_vis",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_googletest(
    name = "road_path_test",
    deps = [
        ":road_path",
        "//drake/automotive/maliput/dragway",
        "//drake/automotive/maliput/monolane",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "pure_pursuit_test",
    deps = [
        "//drake/automotive:pure_pursuit",
        "//drake/automotive/maliput/dragway",
    ],
)

drake_cc_googletest(
    name = "pure_pursuit_controller_test",
    deps = [
        "//drake/automotive:pure_pursuit_controller",
        "//drake/automotive/maliput/dragway",
    ],
)

drake_cc_googletest(
    name = "simple_powertrain_test",
    deps = [
        "//drake/automotive:simple_powertrain",
    ],
)

drake_cc_googletest(
    name = "simple_car_to_euler_floating_joint_test",
    deps = [
        "//drake/automotive:simple_car",
    ],
)

drake_cc_googletest(
    name = "calc_smooth_acceleration_test",
    deps = [
        "//drake/automotive:calc_smooth_acceleration",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "trajectory_car_test",
    deps = [
        "//drake/automotive:trajectory_car",
    ],
)

drake_cc_googletest(
    name = "trajectory_optimization_test",
    deps = [
        ":simple_car",
        "//drake/common:eigen_matrix_compare",
        "//drake/lcm:lcm_call_matlab",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

cpplint()
