# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:eigen_matrix_compare", "@gtest//:main"]
cc_test(
    name = "autodiff_test",
    srcs = ["autodiff_test.cc"],
    size = "small", deps = DEPS + ["//drake/math:autodiff", "//drake/math:gradient"])

cc_test(
    name = "cross_product_test",
    srcs = ["cross_product_test.cc"],
    size = "small", deps = DEPS + ["//drake/math:gradient"])

cc_test(
    name = "expmap_test",
    srcs = ["expmap_test.cc"],
    size = "small", deps = DEPS + ["//drake/math:expmap"])

cc_test(
    name = "jacobian_test",
    srcs = ["jacobian_test.cc"],
    size = "small", deps = DEPS + ["//drake/math:jacobian", "//drake/math:gradient"])

# TODO(jwnimmer-tri) These tests are not yet part of Bazel,
# because they depend on "util":
#     srcs = ["normalize_vector_test.cc"],
#     srcs = ["rotation_conversion_test.cc"],
