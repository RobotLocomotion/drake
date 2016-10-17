# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

# TODO(jwnimmer-tri) Re-test and get this working.  Approxmate answer:
# genrule(
#     name = "generate_drake_path_cc",
#     srcs = ["drake_path.cc.in"],
#     outs = ["drake_path.cc"],
#     cmd = "sed 's#@PROJECT_SOURCE_DIR@#???#g' $< > $@",
#     visibility = ["//visibility:public"],
# )

cc_library(
    name = "common",
    hdrs = [
        "constants.h",
        "drake_assert.h",
        "drake_deprecated.h",
        "drake_export.h",
        "drake_gcc48.h",
        "drake_path.h",
        "drake_throw.h",
        "eigen_autodiff_types.h",
        "eigen_matrix_compare.h",
        "eigen_stl_types.h",
        "eigen_types.h",
        "functional_form.h",
        "nice_type_name.h",
        "number_traits.h",
        "polynomial.h",
        "sorted_vectors_have_intersection.h",
        "text_logging.h",
        "trig_poly.h",
    ],
    srcs = [
        "drake_assert.cc",
        "drake_throw.cc",
        "functional_form.cc",
        "nice_type_name.cc",
        "polynomial.cc",
        "text_logging.cc",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)
