# -*- python -*-

cc_library(
    name = "math",
    hdrs = [
        "autodiff_gradient.h",
        "autodiff.h",
        "axis_angle.h",
        "cross_product.h",
        "cylindrical.h",
        "gradient.h",
        "quaternion.h",
        "random_rotation.h",
        "roll_pitch_yaw.h",
        "rotation_matrix.h",
    ],
    srcs = [
        "autodiff.cc",
        "autodiff_gradient.cc",
        "axis_angle.cc",
        "cross_product.cc",
        "cylindrical.cc",
        "gradient.cc",
        "quaternion.cc",
        "random_rotation.cc",
        "roll_pitch_yaw.cc",
        "rotation_matrix.cc",
    ],
    visibility = ["//visibility:public"],
    deps = ["//drake/common"],
)
