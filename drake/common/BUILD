# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

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
    linkstatic = 1,
    deps = [
        "@eigen//:eigen",
        "@spdlog//:spdlog",
    ],
)

# Specific traits and operators that are relevant to all ScalarTypes.
cc_library(
    name = "cond",
    hdrs = ["cond.h"],
    linkstatic = 1,
)

cc_library(
    name = "number_traits",
    hdrs = ["number_traits.h"],
    linkstatic = 1,
)

# Drake's specific ScalarType-providing libraries.
cc_library(
    name = "double",
    srcs = ["double_overloads.cc"],
    hdrs = ["double_overloads.h"],
    linkstatic = 1,
)

cc_library(
    name = "autodiff",
    hdrs = [
        "autodiff_overloads.h",
        "eigen_autodiff_types.h",
    ],
    linkstatic = 1,
    deps = [
        ":common",
        ":cond",
    ],
)

cc_library(
    name = "functional_form",
    srcs = ["functional_form.cc"],
    hdrs = ["functional_form.h"],
    linkstatic = 1,
    deps = [":common"],
)

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = [
        "polynomial.h",
        "trig_poly.h",
    ],
    linkstatic = 1,
    deps = [
        ":autodiff",
        ":common",
    ],
)

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
    linkstatic = 1,
    deps = [
        ":common",
        ":cond",
        ":number_traits",
    ],
)

# Miscellaneous utilities.
cc_library(
    name = "drake_path",
    # TODO(david-german-tri): Set testonly = 1 once the non-test uses of
    # drake_path are removed.
    # testonly = 1,
    srcs = ["drake_path.cc"],
    hdrs = ["drake_path.h"],
    linkstatic = 1,
    deps = [":common"],
)

# TODO(jwnimmer-tri) GetDrakePath has a long and storied history (#1471, #2174).
# It serves multiple purposes (unit tests loading their models; installed demos
# loading their models; etc.) but doesn't really do any of them well.  The rule
# here in Bazel is intended as a temporary shim, in order to support Bazel and
# CMake build systems concurrently.  We should endeavor to remove this rule (in
# lieu of properly-declared data dependencies) once we are unchained from CMake.
genrule(
    name = "drake_path_genrule",
    srcs = ["drake_path.cc.in"],
    outs = ["drake_path.cc"],
    # TODO(jwnimmer-tri) This should be an abspath, not relpath.
    cmd = "sed 's#@PROJECT_SOURCE_DIR@#drake#g' $< > $@",
)

cc_library(
    name = "eigen_matrix_compare",
    testonly = 1,
    hdrs = ["eigen_matrix_compare.h"],
    linkstatic = 1,
    deps = [":common"],
)

cc_library(
    name = "is_approx_equal_abstol",
    hdrs = ["is_approx_equal_abstol.h"],
    linkstatic = 1,
    deps = [":common"],
)

cc_library(
    name = "nice_type_name",
    srcs = ["nice_type_name.cc"],
    hdrs = ["nice_type_name.h"],
    linkstatic = 1,
    deps = [":common"],
)

cc_library(
    name = "sorted_vectors_have_intersection",
    hdrs = ["sorted_vectors_have_intersection.h"],
    linkstatic = 1,
    deps = [":common"],
)

cc_library(
    name = "text_logging_gflags",
    hdrs = ["text_logging_gflags.h"],
    # TODO(jwnimmer-tri) Ideally, gflags BUILD would do this for us.  Figure
    # out what's going on.  Definitely don't let "-pthread" get copy-pasta'd
    # throughout our code.
    linkopts = ["-pthread"],
    linkstatic = 1,
    deps = [
        ":common",
        "@gflags//:gflags",
    ],
)
