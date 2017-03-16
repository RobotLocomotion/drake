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
        "gen/endless_road_car_config.cc",
        "gen/endless_road_car_state.cc",
        "gen/endless_road_oracle_output.cc",
        "gen/euler_floating_joint_state.cc",
        "gen/idm_planner_parameters.cc",
        "gen/maliput_railcar_config.cc",
        "gen/maliput_railcar_state.cc",
        "gen/simple_car_config.cc",
        "gen/simple_car_state.cc",
    ],
    hdrs = [
        "gen/bicycle_car_parameters.h",
        "gen/bicycle_car_state.h",
        "gen/driving_command.h",
        "gen/endless_road_car_config.h",
        "gen/endless_road_car_state.h",
        "gen/endless_road_oracle_output.h",
        "gen/euler_floating_joint_state.h",
        "gen/idm_planner_parameters.h",
        "gen/maliput_railcar_config.h",
        "gen/maliput_railcar_state.h",
        "gen/simple_car_config.h",
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
        "gen/endless_road_car_config_translator.cc",
        "gen/endless_road_car_state_translator.cc",
        "gen/endless_road_oracle_output_translator.cc",
        "gen/euler_floating_joint_state_translator.cc",
        "gen/maliput_railcar_config_translator.cc",
        "gen/maliput_railcar_state_translator.cc",
        "gen/simple_car_config_translator.cc",
        "gen/simple_car_state_translator.cc",
    ],
    hdrs = [
        "gen/bicycle_car_parameters_translator.h",
        "gen/bicycle_car_state_translator.h",
        "gen/driving_command_translator.h",
        "gen/endless_road_car_config_translator.h",
        "gen/endless_road_car_state_translator.h",
        "gen/endless_road_oracle_output_translator.h",
        "gen/euler_floating_joint_state_translator.h",
        "gen/maliput_railcar_config_translator.h",
        "gen/maliput_railcar_state_translator.h",
        "gen/simple_car_config_translator.h",
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
    name = "curve2",
    srcs = ["curve2.cc"],
    hdrs = ["curve2.h"],
    deps = [
        ":generated_vectors",
    ],
)

drake_cc_library(
    name = "endless_road_car",
    srcs = ["dev/endless_road_car.cc"],
    hdrs = [
        "dev/endless_road_car.h",
        "dev/endless_road_car_to_euler_floating_joint.h",
    ],
    deps = [
        ":generated_vectors",
        ":infinite_circuit_road",
    ],
)

drake_cc_library(
    name = "endless_road_oracle",
    srcs = [
        "dev/endless_road_oracle.cc",
        "dev/endless_road_oracle-internal.h",
    ],
    hdrs = ["dev/endless_road_oracle.h"],
    deps = [
        ":generated_vectors",
        ":infinite_circuit_road",
    ],
)

drake_cc_library(
    name = "linear_car",
    srcs = ["linear_car.cc"],
    hdrs = ["linear_car.h"],
    deps = [
        ":generated_vectors",
        "//drake/common:symbolic",
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
    name = "maliput_railcar",
    srcs = ["maliput_railcar.cc"],
    hdrs = ["maliput_railcar.h"],
    deps = [
        ":calc_smooth_acceleration",
        ":generated_vectors",
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
    name = "single_lane_ego_and_agent",
    srcs = ["single_lane_ego_and_agent.cc"],
    hdrs = ["single_lane_ego_and_agent.h"],
    deps = [
        ":idm_planner",
        ":linear_car",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_library(
    name = "calc_smooth_acceleration",
    srcs = ["calc_smooth_acceleration.cc"],
    hdrs = ["calc_smooth_acceleration.h"],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:symbolic",
    ],
    visibility=[],
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
        ":endless_road_car",
        ":endless_road_oracle",
        ":generated_translators",
        ":generated_vectors",
        ":simple_car",
        ":trajectory_car",
        "//drake/automotive/maliput/api",
        "//drake/automotive/maliput/utility",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/rendering:pose_aggregator",
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
        "//drake/automotive/maliput/dragway",
        "//drake/common:text_logging_gflags",
    ],
)

drake_cc_library(
    name = "infinite_circuit_road",
    srcs = ["dev/infinite_circuit_road.cc"],
    hdrs = ["dev/infinite_circuit_road.h"],
    visibility = [],
    deps = [
        "//drake/automotive/maliput/api",
    ],
)

drake_cc_binary(
    name = "dev_demo",
    srcs = [
        "create_trajectory_params.cc",
        "create_trajectory_params.h",
        "dev/dev_demo.cc",
    ],
    data = [
        ":models",
    ],
    deps = [
        ":automotive_simulator",
        ":infinite_circuit_road",
        "//drake/automotive/maliput/dragway",
        "//drake/automotive/maliput/monolane:loader",
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

village_demo_path = """\
start:l:Express_WN_a,\
l:Express_WN_b,\
l:Express_N1,\
l:Express_NRamp,\
l:Y-Ave_isA,\
l:Y-Ave_blk1,\
l:B2-Y1,\
l:B-St_blk2,\
l:B2-Z1,\
l:Z-Ave_blk1,\
l:A2-Z1,\
l:A-St_blk2,\
l:A-St_isY,\
l:A-St_blk1,\
l:A1-X1,\
l:X-Ave_blk1,\
l:X-Ave_isB,\
l:X-Ave_blk2,\
l:C1-X2,\
l:C-St_blk1,\
l:C1-Y2,\
l:Y-Ave_blk2,\
l:B1-Y2,\
l:B-St_blk1,\
l:B-St_isX,\
l:Express_WRamp_a,\
l:Express_WRamp_b"""

py_binary(
    name = "village-demo",
    srcs = [
        "automotive_demo.py",
    ],
    args = [
        "-road_file $(location //drake/automotive/maliput/monolane:village.yaml)",
        "-num_ado_car=15",
        "-use_ego_car=false",
        "-use_idm",
        "-road_path " + village_demo_path,
    ],
    data = [
        ":dev_demo",
        ":lcm-spy",
        ":steering_command_driver",
        "//drake/automotive/maliput/monolane:village.yaml",
        "@drake_visualizer//:drake-visualizer",
        "@lcm//:lcm-logger",
    ],
    main = "automotive_demo.py",
    deps = [
        "@lcm//:lcm-python",
    ],
)

merging_demo_path = """\
start:l:Express_WN_a,\
l:Express_WN_b,\
l:Express_N1,\
l:Express_NRamp,\
l:A1-YR,\
l:A-St_blk1,\
l:A1-X1,\
l:X-Ave_blk1,\
l:X-Ave_isB,\
l:X-Ave_blk2,\
l:C1-X2,\
l:C-St_blk1,\
l:C1-YR,\
l:Express_SRamp_a,\
l:Express_SRamp_b,\
l:Express_S2,\
l:Express_SW_a,\
l:Express_SW_b,\
l:Express_W_a,\
l:Express_W_b,\
l:Express_WN_a,\
l:Express_WN_b,\
l:Express_N1,\
l:Express_NRamp,\
l:Y-Ave_isA,\
l:Y-Ave_blk1,\
l:B1-Y1,\
l:B-St_blk1,\
l:B-St_isX,\
l:Express_WRamp_a,\
l:Express_WRamp_b"""

py_binary(
    name = "merging-demo",
    srcs = [
        "automotive_demo.py",
    ],
    args = [
        "-road_file $(location //drake/automotive/maliput/monolane:village.yaml)",
        "-num_ado_car=15",
        "-use_ego_car=false",
        "-use_idm",
        "-road_path " + merging_demo_path,
    ],
    data = [
        ":dev_demo",
        ":lcm-spy",
        ":steering_command_driver",
        "//drake/automotive/maliput/monolane:village.yaml",
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
    name = "bicycle_car_test",
    deps = [
        "//drake/automotive:bicycle_car",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "curve2_test",
    deps = [
        "//drake/automotive:curve2",
    ],
)

drake_cc_googletest(
    name = "endless_road_car_test",
    srcs = ["dev/test/endless_road_car_test.cc"],
    deps = [
        "//drake/automotive:endless_road_car",
        "//drake/automotive/maliput/monolane:builder",
        "//drake/automotive/maliput/utility",
    ],
)

drake_cc_googletest(
    name = "endless_road_oracle_test",
    srcs = ["dev/test/endless_road_oracle_test.cc"],
    deps = [
        "//drake/automotive:endless_road_oracle",
        "//drake/automotive/maliput/monolane:builder",
        "//drake/automotive/maliput/utility",
    ],
)

drake_cc_googletest(
    name = "endless_road_car_to_euler_floating_joint_test",
    srcs = ["dev/test/endless_road_car_to_euler_floating_joint_test.cc"],
    deps = [
        "//drake/automotive:endless_road_car",
        "//drake/automotive/maliput/monolane:builder",
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
    name = "single_lane_ego_and_agent_test",
    deps = [
        "//drake/automotive:single_lane_ego_and_agent",
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
    name = "infinite_circuit_road_test",
    srcs = ["dev/test/infinite_circuit_road_test.cc"],
    deps = [
        ":infinite_circuit_road",
        "//drake/automotive/maliput/monolane:builder",
    ],
)

cpplint()
