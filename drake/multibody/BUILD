# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "global_inverse_kinematics",
    srcs = [
        "global_inverse_kinematics.cc",
    ],
    hdrs = [
        "global_inverse_kinematics.h",
    ],
    deps = [
        ":rigid_body_tree",
        "//drake/solvers:mathematical_program",
        "//drake/solvers:rotation_constraint",
    ],
)

drake_cc_library(
    name = "inverse_kinematics",
    srcs = [
        "constraint_wrappers.cc",
        "ik_options.cc",
        "ik_trajectory_helper.cc",
        "inverse_kinematics.cc",
        "inverse_kinematics_backend.cc",
        "inverse_kinematics_pointwise.cc",
        "inverse_kinematics_trajectory.cc",
        "inverse_kinematics_trajectory_backend.cc",
    ],
    hdrs = [
        "constraint_wrappers.h",
        "ik_options.h",
        "ik_trajectory_helper.h",
        "inverse_kinematics_backend.h",
        "rigid_body_ik.h",
    ],
    deps = [
        ":rigid_body_constraint",
        ":rigid_body_tree",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_library(
    name = "rigid_body_constraint",
    srcs = ["constraint/rigid_body_constraint.cc"],
    hdrs = ["constraint/rigid_body_constraint.h"],
    deps = [
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "kinematics_cache",
    srcs = ["kinematics_cache.cc"],
    hdrs = [
        "kinematics_cache.h",
        "kinematics_cache-inl.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/multibody/joints",
    ],
)

drake_cc_library(
    name = "rigid_body",
    srcs = ["rigid_body.cc"],
    hdrs = ["rigid_body.h"],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/multibody/collision",
        "//drake/multibody/joints",
        "//drake/util",
    ],
)

drake_cc_library(
    name = "rigid_body_actuator",
    srcs = ["rigid_body_actuator.cc"],
    hdrs = ["rigid_body_actuator.h"],
    deps = [
        ":rigid_body",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "rigid_body_frame",
    srcs = ["rigid_body_frame.cc"],
    hdrs = ["rigid_body_frame.h"],
    deps = [
        ":rigid_body",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_library(
    name = "rigid_body_loop",
    srcs = ["rigid_body_loop.cc"],
    hdrs = ["rigid_body_loop.h"],
    deps = [
        ":rigid_body_frame",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "rigid_body_tree_datatypes",
    srcs = [],
    hdrs = [
        "force_torque_measurement.h",
        "kinematic_path.h",
        "material_map.h",
        "pose_map.h",
    ],
    visibility = [],
    deps = [
        "//drake/common",
    ],
)

# The rigid_body_tree.h implementation is in two files: rigid_body_tree.cc and
# rigid_body_tree_contact.cc.  This is the rule for the first file.
drake_cc_library(
    name = "rigid_body_tree_cc",
    srcs = ["rigid_body_tree.cc"],
    hdrs = ["rigid_body_tree.h"],
    visibility = [],
    deps = [
        ":kinematics_cache",
        ":rigid_body",
        ":rigid_body_actuator",
        ":rigid_body_frame",
        ":rigid_body_loop",
        ":rigid_body_tree_datatypes",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/multibody/collision",
        "//drake/multibody/joints",
        "//drake/multibody/shapes",
        "//drake/util",
    ],
)

# The rigid_body_tree.h implementation is in two files: rigid_body_tree.cc and
# rigid_body_tree_contact.cc.  This is the rule for the second file.
drake_cc_library(
    name = "rigid_body_tree_contact",
    srcs = ["rigid_body_tree_contact.cc"],
    hdrs = ["rigid_body_tree.h"],
    visibility = [],
    deps = [
        ":kinematics_cache",
        ":rigid_body",
        ":rigid_body_actuator",
        ":rigid_body_frame",
        ":rigid_body_loop",
        ":rigid_body_tree_datatypes",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/math:geometric_transform",
        "//drake/multibody/collision",
        "//drake/multibody/joints",
        "//drake/multibody/shapes",
    ],
)

# This provides a public label for the whole RigidBodyTree.
drake_cc_library(
    name = "rigid_body_tree",
    deps = [
        ":rigid_body_tree_cc",
        ":rigid_body_tree_contact",
    ],
)

drake_cc_library(
    name = "rigid_body_tree_construction",
    srcs = ["rigid_body_tree_construction.cc"],
    hdrs = ["rigid_body_tree_construction.h"],
    deps = [
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "rigid_body_compare_to_clone",
    testonly = 1,
    srcs = ["test/rigid_body_compare_to_clone.cc"],
    hdrs = ["test/rigid_body_compare_to_clone.h"],
    deps = [
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "rigid_body_frame_compare_to_clone",
    testonly = 1,
    srcs = ["test/rigid_body_frame_compare_to_clone.cc"],
    hdrs = ["test/rigid_body_frame_compare_to_clone.h"],
    deps = [
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "rigid_body_tree_compare_to_clone",
    testonly = 1,
    srcs = ["test/rigid_body_tree/rigid_body_tree_compare_to_clone.cc"],
    hdrs = ["test/rigid_body_tree/rigid_body_tree_compare_to_clone.h"],
    deps = [
        ":rigid_body_actuator_compare_to_clone",
        ":rigid_body_compare_to_clone",
        ":rigid_body_frame_compare_to_clone",
        ":rigid_body_loop_compare_to_clone",
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "rigid_body_actuator_compare_to_clone",
    testonly = 1,
    srcs = ["test/rigid_body_actuator_compare_to_clone.cc"],
    hdrs = ["test/rigid_body_actuator_compare_to_clone.h"],
    deps = [
        ":rigid_body_compare_to_clone",
        ":rigid_body_tree",
    ],
)

drake_cc_library(
    name = "rigid_body_loop_compare_to_clone",
    testonly = 1,
    srcs = ["test/rigid_body_loop_compare_to_clone.cc"],
    hdrs = ["test/rigid_body_loop_compare_to_clone.h"],
    deps = [
        ":rigid_body_frame_compare_to_clone",
        ":rigid_body_tree",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
drake_cc_binary(
    name = "benchmark_rigid_body_tree",
    testonly = 1,
    srcs = ["test/benchmark_rigid_body_tree.cc"],
    deps = [
        ":rigid_body_tree",
        "//drake/common:measure_execution",
        "//drake/multibody/parsers",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
drake_cc_binary(
    name = "debug_manipulator_dynamics",
    testonly = 1,
    srcs = ["test/debug_manipulator_dynamics.cc"],
    deps = [
        ":rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rbt_collisions_test",
    srcs = ["test/rigid_body_tree/rbt_collisions_test.cc"],
    data = [":test_models"],
    deps = [
        ":rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_collision_clique_test",
    srcs = ["test/rigid_body_tree/rigid_body_collision_clique_test.cc"],
    deps = [
        ":rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_constraint_test",
    srcs = ["constraint/test/rigid_body_constraint_test.cc"],
    data = ["//drake/examples/Atlas:models"],
    deps = [
        ":rigid_body_constraint",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_tree_kinematics_test",
    srcs = ["test/rigid_body_tree/rigid_body_tree_kinematics_test.cc"],
    data = [
        "test/rigid_body_tree/two_dof_robot.urdf",
        "//drake/multibody/benchmarks:acrobot_models",
    ],
    deps = [
        ":rigid_body_tree",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/benchmarks:acrobot",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_tree_dynamics_test",
    srcs = ["test/rigid_body_tree/rigid_body_tree_dynamics_test.cc"],
    data = ["//drake/examples/Atlas:models"],
    deps = [
        ":rigid_body_tree",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:jacobian",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_tree_clone_test",
    srcs = ["test/rigid_body_tree/rigid_body_tree_clone_test.cc"],
    data = [
        ":test_models",
        "//drake/automotive:models",
        "//drake/examples/Atlas:models",
        "//drake/examples/Pendulum:models",
        "//drake/examples/Valkyrie:models",
    ],
    deps = [
        ":rigid_body_tree_compare_to_clone",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
    ],
)

drake_cc_googletest(
    name = "rigid_body_tree_creation_test",
    srcs = ["test/rigid_body_tree/rigid_body_tree_creation_test.cc"],
    data = glob(["test/rigid_body_tree/*.urdf"]),
    deps = [
        ":rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_tree_face_extraction_test",
    srcs = ["test/rigid_body_tree/rigid_body_tree_face_extraction_test.cc"],
    data = [
        "test/rigid_body_tree/small_sphere_on_large_box.sdf",
    ],
    deps = [
        ":rigid_body_tree",
        "//drake/common:drake_path",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "test_kinematics_cache_checks",
    data = ["//drake/examples/Atlas:models"],
    deps = [
        ":rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "test_ik",
    data = [
        "//drake/examples/Atlas:models",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    deps = [
        ":inverse_kinematics",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "test_ik_pointwise",
    data = ["//drake/examples/Atlas:models"],
    deps = [
        ":inverse_kinematics",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "test_ik_more_constraints",
    data = ["//drake/examples/Atlas:models"],
    tags = ["snopt"],
    deps = [
        ":inverse_kinematics",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "test_ik_traj",
    data = ["//drake/examples/Atlas:models"],
    tags = ["snopt"],
    deps = [
        ":inverse_kinematics",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_test",
    deps = [
        ":rigid_body_compare_to_clone",
        ":rigid_body_tree",
    ],
)

drake_cc_googletest(
    name = "rigid_body_frame_test",
    deps = [
        ":rigid_body_frame_compare_to_clone",
        ":rigid_body_tree",
    ],
)

filegroup(
    name = "test_models",
    testonly = 1,
    srcs = glob([
        "test/**/*.obj",
        "test/**/*.sdf",
        "test/**/*.urdf",
        "test/**/*.xml",
    ]),
)

filegroup(
    name = "models",
    srcs = [
        "models/box.urdf",
        "models/box_small.urdf",
    ],
)

cpplint(data = [
    "constraint/CPPLINT.cfg",
])
