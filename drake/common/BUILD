# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

load("//tools:drake.bzl", "drake_component", "drake_library")

# A library of things that EVERYONE should want and MUST EAT.
# Be appropriately hesitant when adding new things here.
drake_library(
    name = "common",
    components = [
        "drake_assert",
        "drake_throw",
        "text_logging",
    ],
    hdrs = [
        "constants.h",
        "drake_deprecated.h",
        "drake_export.h",
        "drake_gcc48.h",
        "eigen_stl_types.h",
        "eigen_types.h",
        "number_traits.h",
    ],
    deps = ["@eigen//:eigen"],
)

# Drake's ScalarType libraries.
DEPS = [":common"]
drake_library(
    name = "autodiff",
    hdrs = ["eigen_autodiff_types.h"],
    deps = DEPS,
)
drake_library(
    name = "functional_form",
    components = ["functional_form"],
    deps = DEPS,
)
drake_library(
    name = "polynomial",
    components = ["polynomial"],
    hdrs = ["trig_poly.h"],
    deps = DEPS + [":autodiff"],
)
drake_library(
    name = "symbolic",
    components = [
        "symbolic_environment",
        "symbolic_expression",
        "symbolic_formula",
        "symbolic_variable",
        "symbolic_variables",
    ],
    hdrs = ["hash_combine.h"],
    deps = DEPS,
)

# Miscellaneous utilities.
drake_component("drake_path", srcs=[], testonly=1, deps = DEPS)
drake_component("eigen_matrix_compare", srcs=[], testonly=1, deps = DEPS)
drake_component("nice_type_name", deps = DEPS)
drake_component("sorted_vectors_have_intersection", srcs=[], deps = DEPS)

# TODO(jwnimmer-tri) Re-test and get this working.  Approxmate answer:
# genrule(
#     name = "generate_drake_path_cc",
#     srcs = ["drake_path.cc.in"],
#     outs = ["drake_path.cc"],
#     cmd = "sed 's#@PROJECT_SOURCE_DIR@#???#g' $< > $@",
#     visibility = ["//visibility:public"],
# )

# TODO(jwnimmer-tri) These source files are not yet part of Bazel:
# text_logging_gflags.h ... need to add gflags dependency first.
