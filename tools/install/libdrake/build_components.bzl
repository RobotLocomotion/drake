# -*- python -*-

# Should include everything any consumer of Drake would ever need.
#
# Do not update this list by hand; instead, run build_components_refresh.py.
LIBDRAKE_COMPONENTS = [
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
    "//automotive",
    "//automotive/maliput/api",
    "//automotive/maliput/dragway",
    "//automotive/maliput/geometry_base",
    "//automotive/maliput/multilane",
    "//automotive/maliput/rndf",
    "//automotive/maliput/simple_phase_provider",
    "//automotive/maliput/simplerulebook",
    "//automotive/maliput/utility",
    "//common",
    "//common/proto",
    "//common/trajectories",
    "//common:drake_marker_shared_library",  # unpackaged
    "//common:text_logging_gflags_h",  # unpackaged
    "//examples/manipulation_station:manipulation_station",  # unpackaged
    "//geometry",
    "//geometry/dev",
    "//geometry/dev/render",
    "//geometry/dev/render/shaders",
    "//geometry/query_results",
    "//lcm",
    "//manipulation/dev:remote_tree_viewer_wrapper",  # unpackaged
    "//manipulation/perception",
    "//manipulation/planner",
    "//manipulation/scene_generation:random_clutter_generator",  # unpackaged
    "//manipulation/scene_generation:simulate_plant_to_rest",  # unpackaged
    "//manipulation/schunk_wsg",
    "//manipulation/sensors",
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
    "//multibody/multibody_tree",
    "//multibody/multibody_tree/implicit_stribeck:implicit_stribeck_solver",  # unpackaged  # noqa
    "//multibody/multibody_tree/math",
    "//multibody/multibody_tree/multibody_plant",
    "//multibody/multibody_tree/multibody_plant:contact_results_to_lcm",  # unpackaged  # noqa
    "//multibody/multibody_tree/parsing",
    "//perception",
    "//solvers",
    "//systems/analysis",
    "//systems/controllers",
    "//systems/controllers/plan_eval",
    "//systems/controllers/qp_inverse_dynamics",
    "//systems/estimators",
    "//systems/framework",
    "//systems/lcm",
    "//systems/plants/spring_mass_system",
    "//systems/primitives",
    "//systems/rendering",
    "//systems/robotInterfaces",
    "//systems/sensors",
    "//systems/sensors/dev",
    "//systems/trajectory_optimization",
    "//util",
    # //examples/atlas:atlas_util (indirectly)
]
