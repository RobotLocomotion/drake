# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autodiff",
    srcs = ["autodiff.cc"],
    hdrs = ["autodiff.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

cc_library(
    name = "continuous_algebraic_ricatti_equation",
    srcs = ["continuous_algebraic_ricatti_equation.cc"],
    hdrs = ["continuous_algebraic_ricatti_equation.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:is_approx_equal_abstol",
    ],
)

# TODO(jwnimmer-tri) Improved name for this library, "pose_representations"?
cc_library(
    name = "geometric_transform",
    srcs = [
        "axis_angle.cc",
        "quaternion.cc",
        "random_rotation.cc",
        "roll_pitch_yaw.cc",
        "rotation_matrix.cc",
    ],
    hdrs = [
        "axis_angle.h",
        "quaternion.h",
        "random_rotation.h",
        "roll_pitch_yaw.h",
        "roll_pitch_yaw_not_using_quaternion.h",
        "roll_pitch_yaw_using_quaternion.h",
        "rotation_matrix.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "gradient",
    srcs = [
        "autodiff_gradient.cc",
        "cross_product.cc",
        "gradient.cc",
        "normalize_vector.cc",
        "rotation_conversion_gradient.cc",
    ],
    hdrs = [
        "autodiff_gradient.h",
        "cross_product.h",
        "gradient.h",
        "gradient_util.h",
        "normalize_vector.h",
        "rotation_conversion_gradient.h",
    ],
    linkstatic = 1,
    deps = [
        ":autodiff",
        "//drake/common",
    ],
)

cc_library(
    name = "matrix_util",
    srcs = ["matrix_util.cc"],
    hdrs = ["matrix_util.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:number_traits",
    ],
)

# These all require bleeding-edge Eigen, so please don't fold them into any
# other important labels until we get all workspaces on newer-Eigen.
cc_library(
    name = "eigen_sparse_triplet",
    srcs = ["eigen_sparse_triplet.cc"],
    hdrs = ["eigen_sparse_triplet.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "expmap",
    srcs = ["expmap.cc"],
    hdrs = ["expmap.h"],
    linkstatic = 1,
    deps = [
        ":gradient",
        "//drake/common",
    ],
)

cc_library(
    name = "jacobian",
    srcs = ["jacobian.cc"],
    hdrs = ["jacobian.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

# === test/ ===

cc_googletest(
    name = "autodiff_test",
    deps = [
        ":autodiff",
        ":gradient",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "cross_product_test",
    deps = [
        ":gradient",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "expmap_test",
    deps = [
        ":expmap",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "eigen_sparse_triplet_test",
    deps = [
        ":eigen_sparse_triplet",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "jacobian_test",
    deps = [
        ":gradient",
        ":jacobian",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "gradient_util_test",
    deps = [
        ":gradient",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "matrix_util_test",
    deps = [
        ":matrix_util",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:symbolic",
    ],
)

cc_googletest(
    name = "normalize_vector_test",
    deps = [
        ":gradient",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "rotation_conversion_test",
    deps = [
        ":geometric_transform",
        ":gradient",
        "//drake/common:eigen_matrix_compare",
    ],
)
