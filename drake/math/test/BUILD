# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/common:eigen_matrix_compare",
    "@gtest//:main",
]

cc_test(
    name = "autodiff_test",
    size = "small",
    srcs = ["autodiff_test.cc"],
    deps = DEPS + [
        "//drake/math:autodiff",
        "//drake/math:gradient",
    ],
)

cc_test(
    name = "cross_product_test",
    size = "small",
    srcs = ["cross_product_test.cc"],
    deps = DEPS + ["//drake/math:gradient"],
)

cc_test(
    name = "expmap_test",
    size = "small",
    srcs = ["expmap_test.cc"],
    deps = DEPS + ["//drake/math:expmap"],
)

cc_test(
    name = "eigen_sparse_triplet_test",
    size = "small",
    srcs = ["eigen_sparse_triplet_test.cc"],
    deps = DEPS + ["//drake/math:eigen_sparse_triplet"],
)

cc_test(
    name = "jacobian_test",
    size = "small",
    srcs = ["jacobian_test.cc"],
    deps = DEPS + [
        "//drake/math:jacobian",
        "//drake/math:gradient",
    ],
)

cc_test(
    name = "gradient_util_test",
    size = "small",
    srcs = ["gradient_util_test.cc"],
    deps = DEPS + ["//drake/math:gradient"],
)

cc_test(
    name = "normalize_vector_test",
    size = "small",
    srcs = ["normalize_vector_test.cc"],
    deps = DEPS + ["//drake/math:gradient"],
)

cc_test(
    name = "rotation_conversion_test",
    size = "small",
    srcs = ["rotation_conversion_test.cc"],
    deps = DEPS + [
        "//drake/math:gradient",
        "//drake/math:geometric_transform",
    ],
)
