# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

# A library of things that EVERYONE should want and MUST EAT.
# Be appropriately hesitant when adding new things here.
cc_library(
    name = "common",
    srcs = [
        "drake_assert.cc",
        "drake_throw.cc",
        "text_logging.cc",
    ],
    hdrs = [
        "constants.h",
        "drake_assert.h",
        "drake_compat.h",
        "drake_deprecated.h",
        "drake_throw.h",
        "eigen_stl_types.h",
        "eigen_types.h",
        "text_logging.h",
    ],
    deps = ["@eigen//:eigen", "@spdlog//:spdlog"],
    linkstatic = 1)

# Specific traits and operators that are relevant to all ScalarTypes.
cc_library(
    name = "cond",
    hdrs = ["cond.h"],
    linkstatic = 1)

cc_library(
    name = "number_traits",
    hdrs = ["number_traits.h"],
    linkstatic = 1)

# Drake's specific ScalarType-providing libraries.
cc_library(
    name = "autodiff",
    hdrs = ["autodiff_overloads.h", "eigen_autodiff_types.h"],
    deps = [":common"],
    linkstatic = 1)

cc_library(
    name = "functional_form",
    srcs = ["functional_form.cc"],
    hdrs = ["functional_form.h"],
    deps = [":common"],
    linkstatic = 1)

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = ["polynomial.h", "trig_poly.h"],
    deps = [":common", ":autodiff"],
    linkstatic = 1)

cc_library(
    name = "symbolic",
    srcs = [
        "symbolic_environment.cc",
        "symbolic_expression.cc",
        "symbolic_expression_cell.cc",
        "symbolic_formula.cc",
        "symbolic_formula_cell.cc",
        "symbolic_variable.cc",
        "symbolic_variables.cc",
    ],
    hdrs = [
        "hash.h",
        "symbolic_environment.h",
        "symbolic_expression.h",
        "symbolic_expression_cell.h",
        "symbolic_formula.h",
        "symbolic_formula_cell.h",
        "symbolic_variable.h",
        "symbolic_variables.h",
    ],
    deps = [":common", ":cond", ":number_traits"],
    linkstatic = 1)

# Miscellaneous utilities.
cc_library(
    name = "drake_path",
    hdrs = ["drake_path.h"],
    testonly = 1,
    deps = [":common"],
    linkstatic = 1)

cc_library(
    name = "eigen_matrix_compare",
    hdrs = ["eigen_matrix_compare.h"],
    testonly = 1,
    deps = [":common"],
    linkstatic = 1)

cc_library(
    name = "nice_type_name",
    srcs = ["nice_type_name.cc"],
    hdrs = ["nice_type_name.h"],
    deps = [":common"],
    linkstatic = 1)

cc_library(
    name = "sorted_vectors_have_intersection",
    hdrs = ["sorted_vectors_have_intersection.h"],
    deps = [":common"],
    linkstatic = 1)

# TODO(jwnimmer-tri) Re-test and get this working.  Approxmate answer:
# genrule(
#     name = "generate_drake_path_cc",
#     srcs = ["drake_path.cc.in"],
#     outs = ["drake_path.cc"],
#     cmd = "sed 's#@PROJECT_SOURCE_DIR@#???#g' $< > $@",
# )

# TODO(jwnimmer-tri) These source files are not yet part of Bazel:
# text_logging_gflags.h ... need to add gflags dependency first.
