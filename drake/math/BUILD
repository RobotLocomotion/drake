# -*- python -*

package(default_visibility = ["//visibility:public"])
DEPS = ["//drake/common:common"]

# TODO(jwnimmer-tri) Seems like this should live in common, with the other
# scalartype stuff?
cc_library(
    name = "autodiff_overloads",
    hdrs = ["autodiff_overloads.h"],
    deps = DEPS + ["//drake/common:autodiff"],
    linkstatic = 1)

cc_library(
    name = "autodiff",
    srcs = ["autodiff.cc"],
    hdrs = ["autodiff.h"],
    deps = DEPS + [":autodiff_overloads"],
    linkstatic = 1)

# TODO(jwnimmer-tri) Imrprove name for this library, "pose_representations"?
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
        "rotation_matrix.h",
    ],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "gradient",
    srcs = [
         "autodiff_gradient.cc",
         "cross_product.cc",
         "gradient.cc",
         # TODO(jwnimmer-tri) These source files are not yet part of Bazel,
         # because they depend on "util":
         # "normalize_vector.h",
         # "rotation_conversion_gradient.h",
    ],
    hdrs = [
         "autodiff_gradient.h",
         "cross_product.h",
         "gradient.h",
    ],
    deps = DEPS + [":autodiff"],
    linkstatic = 1)

# These all require bleeding-edge Eigen, so please don't fold them into any
# other important labels until we get all workspaces on newer-Eigen.
cc_library(
    name = "eigen_sparse_triplet",
    hdrs = ["eigen_sparse_triplet.h"],
    deps = DEPS,
    linkstatic = 1)
cc_library(
    name = "expmap",
    hdrs = ["expmap.h"],
    deps = DEPS + [":gradient"],
    linkstatic = 1)
cc_library(
    name = "jacobian",
    hdrs = ["jacobian.h"],
    deps = DEPS,
    linkstatic = 1)
