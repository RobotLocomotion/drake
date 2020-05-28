# -*- python -*-

# This file governs the contents of libdrake.so.

# Do not update this list by hand; instead, from the drake workspace run
#
#   tools/install/libdrake/build_components_refresh.py
#
# and it will rewrite this file automatically for you.  If the refresh made any
# edits, then `git status` will show this file as modified; in that case, you
# should commit the changes made by the refresh script.
LIBDRAKE_COMPONENTS = [
    "//attic/manipulation/planner",
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
    "//attic/systems/controllers",
    "//attic/systems/rendering",
    "//attic/systems/sensors",
    "//attic/systems/trajectory_optimization",
    "//attic/util",
    "//common",
    "//common/proto",
    "//common/trajectories",
    "//common/yaml",
    "//common:drake_marker_shared_library",  # unpackaged
    "//examples/acrobot:acrobot_geometry",  # unpackaged
    "//examples/acrobot:acrobot_input",  # unpackaged
    "//examples/acrobot:acrobot_params",  # unpackaged
    "//examples/acrobot:acrobot_plant",  # unpackaged
    "//examples/acrobot:acrobot_state",  # unpackaged
    "//examples/acrobot:spong_controller",  # unpackaged
    "//examples/acrobot:spong_controller_params",  # unpackaged
    "//examples/compass_gait:compass_gait",  # unpackaged
    "//examples/compass_gait:compass_gait_geometry",  # unpackaged
    "//examples/compass_gait:compass_gait_vector_types",  # unpackaged
    "//examples/manipulation_station:manipulation_station",  # unpackaged
    "//examples/manipulation_station:manipulation_station_hardware_interface",  # unpackaged  # noqa
    "//examples/pendulum:pendulum_geometry",  # unpackaged
    "//examples/pendulum:pendulum_plant",  # unpackaged
    "//examples/pendulum:pendulum_vector_types",  # unpackaged
    "//examples/quadrotor:quadrotor_geometry",  # unpackaged
    "//examples/quadrotor:quadrotor_plant",  # unpackaged
    "//examples/rimless_wheel:rimless_wheel",  # unpackaged
    "//examples/rimless_wheel:rimless_wheel_geometry",  # unpackaged
    "//examples/rimless_wheel:rimless_wheel_vector_types",  # unpackaged
    "//examples/van_der_pol:van_der_pol",  # unpackaged
    "//geometry",
    "//geometry/proximity",
    "//geometry/query_results",
    "//geometry/render",
    "//geometry/render/gl_renderer",
    "//geometry/render/shaders",
    "//lcm",
    "//manipulation/kuka_iiwa",
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
    "//multibody/hydroelastics",
    "//multibody/inverse_kinematics",
    "//multibody/math",
    "//multibody/optimization",
    "//multibody/parsing",
    "//multibody/plant",
    "//multibody/plant:contact_results_to_lcm",  # unpackaged
    "//multibody/topology:multibody_graph",  # unpackaged
    "//multibody/tree",
    "//multibody/triangle_quadrature",
    "//perception",
    "//solvers",
    "//solvers/fbstab",
    "//solvers/fbstab/components",
    "//systems/analysis",
    "//systems/controllers",
    "//systems/estimators",
    "//systems/framework",
    "//systems/lcm",
    "//systems/optimization",
    "//systems/plants/spring_mass_system",
    "//systems/primitives",
    "//systems/rendering",
    "//systems/sensors",
    "//systems/trajectory_optimization",
    # //attic:attic_warning (indirectly)
    # //common:filesystem (indirectly)
    # //third_party/com_github_finetjul_bender:vtkCapsuleSource (indirectly)
    # //third_party/com_github_jbeder_yaml_cpp:emitfromevents (indirectly)
]
