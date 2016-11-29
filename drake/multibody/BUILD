# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rigid_body_tree",
    srcs = [
        "parser_model_instance_id_table.cc",
        "parser_common.cc",
        "parser_sdf.cc",
        "parser_urdf.cc",
        "rigid_body.cc",
        "rigid_body_actuator.cc",
        "rigid_body_frame.cc",
        "rigid_body_loop.cc",
        "rigid_body_tree.cc",
        "xml_util.cc",
        "collision/drake_collision.cc",
        "collision/element.cc",
        "collision/model.cc",
        "collision/unusable_model.cc",
    ] + glob([
        "joints/*.cc",
        "shapes/*.cc",
    ]),
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
        "collision/drake_collision.h",
        "collision/element.h",
        "collision/model.h",
        "collision/point_pair.h",
        "collision/unusable_model.h",
    ] + glob([
        "joints/*.h",
        "shapes/*.h",
    ]),
    linkstatic = 1,
    deps = [
        "//drake/common:drake_path",
        "//drake/common:sorted_vectors_have_intersection",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/thirdParty:spruce",
        "//drake/thirdParty:tinydir",
        "//drake/thirdParty:tinyxml2",
        "//drake/util",
    ],
)

cc_library(
    name = "drake_visualizer",
    srcs = [
        "rigid_body_plant/drake_visualizer.cc",
        "rigid_body_plant/viewer_draw_translator.cc",
    ],
    hdrs = [
        "rigid_body_plant/drake_visualizer.h",
        "rigid_body_plant/viewer_draw_translator.h",
    ],
    linkstatic = 1,
    deps = [
        ":rigid_body_tree",
        "//drake/lcmtypes:viewer",
        "//drake/systems/lcm",
    ],
)
