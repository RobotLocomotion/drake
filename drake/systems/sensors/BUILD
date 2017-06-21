# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "sensors",
    srcs = [],
    hdrs = [],
    deps = [
        ":accelerometer",
        ":camera_info",
        ":depth_sensor",
        ":rgbd_camera",
        ":rotary_encoders",
    ],
)

drake_cc_library(
    name = "camera_info",
    srcs = [
        "camera_info.cc",
    ],
    hdrs = [
        "camera_info.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "image",
    srcs = [],
    hdrs = [
        "image.h",
        "pixel_types.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common:reinit_after_move",
    ],
)

drake_cc_library(
    name = "image_to_lcm_image_array_t",
    srcs = [
        "image_to_lcm_image_array_t.cc",
    ],
    hdrs = [
        "image_to_lcm_image_array_t.h",
    ],
    deps = [
        ":image",
        "//drake/common",
        "//drake/systems/framework",
        "@lcmtypes_robotlocomotion",
        "@zlib",
    ],
)

drake_cc_library(
    name = "rotary_encoders",
    srcs = ["rotary_encoders.cc"],
    hdrs = ["rotary_encoders.h"],
    deps = [
        "//drake/common:unused",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "accelerometer",
    srcs = [
        "accelerometer.cc",
        "accelerometer_output.cc",
    ],
    hdrs = [
        "accelerometer.h",
        "accelerometer_output.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "depth_sensor",
    srcs = [
        "depth_sensor.cc",
        "depth_sensor_output.cc",
        "depth_sensor_specification.cc",
    ],
    hdrs = [
        "depth_sensor.h",
        "depth_sensor_output.h",
        "depth_sensor_specification.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "depth_sensor_to_lcm_point_cloud_message",
    srcs = [
        "depth_sensor_to_lcm_point_cloud_message.cc",
    ],
    hdrs = [
        "depth_sensor_to_lcm_point_cloud_message.h",
    ],
    deps = [
        ":depth_sensor",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/rendering:pose_vector",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_library(
    name = "gyroscope",
    srcs = [
        "gyroscope.cc",
        "gyroscope_output.cc",
    ],
    hdrs = [
        "gyroscope.h",
        "gyroscope_output.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "vtk_util",
    srcs = ["vtk_util.cc"],
    hdrs = ["vtk_util.h"],
    deps = [
        "//drake/common",
        "@eigen",
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonTransforms",
        "@vtk//:vtkFiltersSources",
    ],
)

drake_cc_library(
    name = "rgbd_camera",
    srcs = ["rgbd_camera.cc"],
    hdrs = ["rgbd_camera.h"],
    deps = [
        ":camera_info",
        ":image",
        ":vtk_util",
        "//drake/math:geometric_transform",
        "//drake/multibody:rigid_body_frame",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
        "//drake/systems/rendering:pose_vector",
        "@eigen",
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonDataModel",
        "@vtk//:vtkCommonTransforms",
        "@vtk//:vtkFiltersGeneral",
        "@vtk//:vtkFiltersSources",
        "@vtk//:vtkIOGeometry",
        "@vtk//:vtkIOImage",
        "@vtk//:vtkRenderingCore",
        "@vtk//:vtkRenderingOpenGL2",
    ],
)

drake_cc_binary(
    name = "rgbd_camera_publish_lcm_example",
    srcs = [
        "rgbd_camera_publish_lcm_example.cc",
    ],
    deps = [
        ":image_to_lcm_image_array_t",
        ":rgbd_camera",
        "//drake/common:unused",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/lcm",
        "//drake/systems/rendering:pose_stamped_t_pose_vector_translator",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_library(
    name = "accelerometer_example_diagram",
    srcs = [
        "test/accelerometer_test/accelerometer_example_diagram.cc",
        "test/accelerometer_test/accelerometer_test_logger.cc",
        "test/accelerometer_test/accelerometer_xdot_hack.cc",
    ],
    hdrs = [
        "test/accelerometer_test/accelerometer_example_diagram.h",
        "test/accelerometer_test/accelerometer_test_logger.h",
        "test/accelerometer_test/accelerometer_xdot_hack.h",
    ],
    data = [
        "//drake/examples/Pendulum:models",
    ],
    deps = [
        ":accelerometer",
        "//drake/common",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_that_publishes_xdot",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
    ],
)

drake_cc_binary(
    name = "accelerometer_example",
    srcs = [
        "test/accelerometer_test/accelerometer_example.cc",
    ],
    deps = [
        ":accelerometer_example_diagram",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "@com_github_gflags_gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "camera_info_test",
    deps = [
        ":camera_info",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "image_test",
    deps = [
        ":image",
    ],
)

drake_cc_googletest(
    name = "rotary_encoders_test",
    deps = [
        ":rotary_encoders",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "accelerometer_test",
    srcs = [
        "test/accelerometer_test/accelerometer_test.cc",
    ],
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":accelerometer_example_diagram",
        "//drake/common:eigen_matrix_compare",
        "//drake/lcm:mock",
        "//drake/systems/analysis:simulator",
    ],
)

drake_cc_googletest(
    name = "depth_sensor_test",
    srcs = ["test/depth_sensor_test.cc"],
    deps = [
        ":depth_sensor",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "depth_sensor_to_lcm_point_cloud_message_test",
    srcs = ["test/depth_sensor_to_lcm_point_cloud_message_test.cc"],
    deps = [
        ":depth_sensor_to_lcm_point_cloud_message",
        "//drake/common:drake_path",
        "//drake/lcm:mock",
    ],
)

drake_cc_googletest(
    name = "gyroscope_test",
    srcs = ["test/gyroscope_test.cc"],
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":gyroscope",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

# TODO(jamiesnape): Remove the manual tag when Xvfb is available on CI.
drake_cc_googletest(
    name = "rgbd_camera_test",
    data = [
        "test/models/box.sdf",
        "test/models/cylinder.sdf",
        "test/models/mesh_box.sdf",
        "test/models/meshes/box.obj",
        "test/models/meshes/box.png",
        "test/models/multiple_visuals.sdf",
        "test/models/nothing.sdf",
        "test/models/sphere.sdf",
        "test/models/three_boxes.sdf",
    ],
    tags = ["manual"],
    deps = [
        ":rgbd_camera",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "vtk_util_test",
    deps = [":vtk_util"],
)

drake_cc_googletest(
    name = "image_to_lcm_image_array_t_test",
    deps = [":image_to_lcm_image_array_t"],
)

cpplint()
