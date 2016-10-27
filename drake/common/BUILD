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

# TODO(jwnimmer-tri) These source files are not yet part of Bazel:
# text_logging_gflags.h

# A library of things that everyone will want and must eat.
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
        "nice_type_name.h",
        "number_traits.h",
        "sorted_vectors_have_intersection.h",
        "text_logging.h",
    ],
    srcs = [
        "drake_assert.cc",
        "drake_throw.cc",
        "nice_type_name.cc",
        "text_logging.cc",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)

cc_library(
    name = "symbolic",
    hdrs = [
        "hash_combine.h",
        "symbolic_environment.h",
        "symbolic_expression.h",
        "symbolic_formula.h",
        "symbolic_variable.h",
        "symbolic_variables.h",
    ],
    srcs = [
        "symbolic_environment.cc",
        "symbolic_expression.cc",
        "symbolic_formula.cc",
        "symbolic_variable.cc",
        "symbolic_variables.cc",
    ],
    visibility = ["//visibility:public"],
    deps = ["common"],
)

cc_library(
    name = "functional_form",
    hdrs = ["functional_form.h"],
    srcs = ["functional_form.cc"],
    visibility = ["//visibility:public"],
    deps = ["common"],
)

cc_library(
    name = "polynomial",
    hdrs = ["polynomial.h", "trig_poly.h"],
    srcs = ["polynomial.cc"],
    visibility = ["//visibility:public"],
    deps = ["common"],
)
