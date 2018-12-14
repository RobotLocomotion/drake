# -*- python -*-

# Should include everything any consumer of Drake would ever need.
#
# Do not update this list by hand; instead, run build_components_refresh.py.
LIBDRAKE_COMPONENTS = [
    "//attic/manipulation/dev:remote_tree_viewer_wrapper",  # unpackaged
    "//attic/manipulation/planner",
    "//attic/manipulation/scene_generation:random_clutter_generator",  # unpackaged  # noqa
    "//attic/manipulation/scene_generation:simulate_plant_to_rest",  # unpackaged  # noqa
    "//attic/manipulation/sensors",
    "//attic/manipulation/util",
    "//attic/multibody",
    "//attic/multibody/collision",
    "//attic/multibody/joints",
    "//attic/multibody/parsers",
    "//attic/multibody/rigid_body_plant:compliant_contact_model",  # unpackaged
    "//attic/multibody/rigid_body_plant:compliant_material",  # unpackaged
    "//attic/multibody/rigid_body_plant:contact_results",  # unpackaged
    "//attic/multibody/rigid_body_plant:contact_results_to_lcm",  # unpackaged
    "//attic/multibody/rigid_body_plant:create_load_robot_message",  # unpackaged  # noqa
    "//attic/multibody/rigid_body_plant:drake_visualizer",  # unpackaged
    "//attic/multibody/rigid_body_plant:frame_visualizer",  # unpackaged
    "//attic/multibody/rigid_body_plant:rigid_body_plant",  # unpackaged
    "//attic/multibody/rigid_body_plant:rigid_body_plant_bridge",  # unpackaged
    "//attic/multibody/shapes",
    "//attic/perception",
    "//attic/systems/controllers",
    "//attic/systems/controllers/plan_eval",
    "//attic/systems/controllers/qp_inverse_dynamics",
    "//attic/systems/rendering",
    "//attic/systems/robotInterfaces",
    "//attic/systems/sensors",
    "//attic/systems/trajectory_optimization",
    "//attic/util",
    "//automotive",
    "//automotive/maliput/api",
    "//automotive/maliput/dragway",
    "//automotive/maliput/geometry_base",
    "//automotive/maliput/multilane",
    "//automotive/maliput/rndf",
    "//automotive/maliput/simple_phase_book",
    "//automotive/maliput/simple_phase_provider",
    "//automotive/maliput/simplerulebook",
    "//automotive/maliput/utility",
    "//common",
    "//common/proto",
    "//common/trajectories",
    "//common:drake_marker_shared_library",  # unpackaged
    "//common:text_logging_gflags_h",  # unpackaged
    "//examples/acrobot:acrobot_input",  # unpackaged
    "//examples/acrobot:acrobot_params",  # unpackaged
    "//examples/acrobot:acrobot_plant",  # unpackaged
    "//examples/acrobot:acrobot_state",  # unpackaged
    "//examples/compass_gait:compass_gait",  # unpackaged
    "//examples/compass_gait:compass_gait_vector_types",  # unpackaged
    "//examples/manipulation_station:manipulation_station",  # unpackaged
    "//examples/manipulation_station:manipulation_station_hardware_interface",  # unpackaged  # noqa
    "//examples/pendulum:pendulum_plant",  # unpackaged
    "//examples/pendulum:pendulum_vector_types",  # unpackaged
    "//examples/rimless_wheel:rimless_wheel",  # unpackaged
    "//examples/rimless_wheel:rimless_wheel_vector_types",  # unpackaged
    "//examples/van_der_pol:van_der_pol",  # unpackaged
    "//geometry",
    "//geometry/dev",
    "//geometry/dev/render",
    "//geometry/dev/render/shaders",
    "//geometry/query_results",
    "//lcm",
    "//manipulation/perception",
    "//manipulation/planner",
    "//manipulation/schunk_wsg",
    "//manipulation/util",
    "//math",
    "//multibody/benchmarks/acrobot",
    "//multibody/benchmarks/free_body",
    "//multibody/benchmarks/inclined_plane",
    "//multibody/benchmarks/kuka_iiwa_robot",
    "//multibody/benchmarks/mass_damper_spring",
    "//multibody/benchmarks/pendulum",
    "//multibody/constraint",
    "//multibody/inverse_kinematics",
    "//multibody/math",
    "//multibody/multibody_tree",
    "//multibody/multibody_tree/implicit_stribeck",
    "//multibody/multibody_tree/math",
    "//multibody/multibody_tree/multibody_plant",
    "//multibody/multibody_tree/parsing",
    "//multibody/parsing",
    "//multibody/plant",
    "//multibody/plant:contact_results_to_lcm",  # unpackaged
    "//multibody/tree",
    "//perception",
    "//solvers",
    "//systems/analysis",
    "//systems/controllers",
    "//systems/estimators",
    "//systems/framework",
    "//systems/lcm",
    "//systems/plants/spring_mass_system",
    "//systems/primitives",
    "//systems/rendering",
    "//systems/sensors",
    "//systems/sensors/dev",
    "//systems/trajectory_optimization",
    # //examples/kuka_iiwa_arm:iiwa_common (indirectly)
    # //examples/kuka_iiwa_arm:iiwa_lcm (indirectly)
    # //multibody/multibody_tree/joints:forwarding_hdrs (indirectly)
]
