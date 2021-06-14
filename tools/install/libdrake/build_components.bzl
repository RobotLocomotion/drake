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
    "//common",
    "//common/proto",
    "//common/schema",
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
    "//geometry/optimization",
    "//geometry/proximity",
    "//geometry/query_results",
    "//geometry/render",
    "//geometry/render/gl_renderer",
    "//geometry/render/shaders",
    "//lcm",
    "//manipulation/kinova_jaco",
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
    "//multibody/contact_solvers",
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
    "//systems/primitives",
    "//systems/rendering",
    "//systems/sensors",
    "//systems/trajectory_optimization",
    # //common:filesystem (indirectly)
    # //common:nice_type_name_override_header (indirectly)
    # //geometry/render/gl_renderer:apple_only_no_render_engine_gl_factory (indirectly)  # noqa
    # //third_party/com_github_finetjul_bender:vtkCapsuleSource (indirectly)
    # //third_party/com_github_jbeder_yaml_cpp:emitfromevents (indirectly)
]
