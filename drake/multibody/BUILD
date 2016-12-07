# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
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
    linkstatic = 1,
    deps = [
        ":rigid_body_constraint",
        ":rigid_body_tree",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

cc_library(
    name = "rigid_body_constraint",
    srcs = ["constraint/rigid_body_constraint.cc"],
    hdrs = ["constraint/rigid_body_constraint.h"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
    ],
)

cc_library(
    name = "rigid_body_tree",
    srcs = [
        "kinematics_cache.cc",
        "parser_common.cc",
        "parser_model_instance_id_table.cc",
        "parser_sdf.cc",
        "parser_urdf.cc",
        "rigid_body.cc",
        "rigid_body_actuator.cc",
        "rigid_body_frame.cc",
        "rigid_body_loop.cc",
        "rigid_body_tree.cc",
        "rigid_body_tree_contact.cc",
        "xml_util.cc",
    ],
    hdrs = [
        "force_torque_measurement.h",
        "kinematic_path.h",
        "kinematics_cache.h",
        "kinematics_cache-inl.h",
        "material_map.h",
        "parser_common.h",
        "parser_model_instance_id_table.h",
        "parser_sdf.h",
        "parser_urdf.h",
        "pose_map.h",
        "rigid_body.h",
        "rigid_body_actuator.h",
        "rigid_body_frame.h",
        "rigid_body_loop.h",
        "rigid_body_tree.h",
        "xml_util.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common:drake_path",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/multibody/collision",
        "//drake/multibody/joints",
        "//drake/multibody/shapes",
        "//drake/thirdParty:spruce",
        "//drake/thirdParty:tinydir",
        "//drake/thirdParty:tinyxml2",
        "//drake/util",
    ],
)

cc_library(
    name = "rigid_body_tree_construction",
    srcs = ["rigid_body_tree_construction.cc"],
    hdrs = ["rigid_body_tree_construction.h"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "benchmark_rigid_body_tree",
    testonly = 1,
    srcs = ["test/benchmark_rigid_body_tree.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "//drake/common/test:measure_execution",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "debug_manipulator_dynamics",
    testonly = 1,
    srcs = ["test/debug_manipulator_dynamics.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_kinematics_test",
    testonly = 1,
    srcs = ["test/urdf_kinematics_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_collision_test",
    testonly = 1,
    srcs = ["test/urdf_collision_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_manipulator_dynamics_test",
    srcs = ["test/urdf_manipulator_dynamics_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "parser_urdf_test",
    size = "small",
    srcs = ["test/parser_urdf_test/parser_urdf_test.cc"],
    data = glob(["test/parser_urdf_test/*.urdf"]),
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rbt_collisions_test",
    size = "small",
    srcs = ["test/rigid_body_tree/rbt_collisions_test.cc"],
    data = [":test_models"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rigid_body_collision_clique_test",
    size = "small",
    srcs = ["test/rigid_body_tree/rigid_body_collision_clique_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rigid_body_constraint_test",
    size = "small",
    srcs = ["constraint/test/rigid_body_constraint_test.cc"],
    data = ["//drake/examples/Atlas:models"],
    linkstatic = 1,
    deps = [
        ":rigid_body_constraint",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rigid_body_tree_inverse_dynamics_test",
    size = "small",
    srcs = ["test/rigid_body_tree/rigid_body_tree_inverse_dynamics_test.cc"],
    data = ["//drake/examples/Atlas:models"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:jacobian",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rigid_body_tree_test",
    size = "small",
    srcs = ["test/rigid_body_tree/rigid_body_tree_test.cc"],
    data = glob(["test/rigid_body_tree/*.urdf"]),
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "test_kinematics_cache_checks",
    size = "small",
    srcs = ["test/test_kinematics_cache_checks.cc"],
    data = ["//drake/examples/Atlas:models"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rigid_body_test",
    size = "small",
    srcs = ["test/rigid_body_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "@gtest//:main",
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
