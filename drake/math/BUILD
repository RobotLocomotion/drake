# -*- python -*

load("//tools:drake.bzl", "drake_component", "drake_library")
DEPS = ["//drake/common:common"]

# TODO(jwnimmer-tri) Seems like this should live in common, with the other
# scalartype stuff?
drake_library(
    name = "autodiff_overloads",
    hdrs = ["autodiff_overloads.h"],
    deps = DEPS + ["//drake/common:autodiff"],
)

drake_library(
    name = "autodiff",
    components = ["autodiff"],
    deps = DEPS + [":autodiff_overloads"],
)

drake_library(
    name = "geometric_transform",
    components = [
        "axis_angle",
        "quaternion",
        "random_rotation",
        "roll_pitch_yaw",
        "rotation_matrix",
    ],
    deps = DEPS,
)

drake_library(
    name = "gradient",
    components = [
         "autodiff_gradient",
         "cross_product",
         "gradient",
         # TODO(jwnimmer-tri) These source files are not yet part of Bazel,
         # because they depend on "util":
         # "normalize_vector",
         # "rotation_conversion_gradient",
    ],
    deps = DEPS + [":autodiff"],
)

# These all require bleeding-edge Eigen, so please don't fold them into any
# other important labels until we get all workspaces on newer-Eigen.
drake_component(name = "eigen_sparse_triplet", deps = DEPS)
drake_component(name = "expmap", deps = DEPS + [":gradient"])
drake_component(name = "jacobian", deps = DEPS)
