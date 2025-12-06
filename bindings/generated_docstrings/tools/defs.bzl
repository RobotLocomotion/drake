load("//tools/skylark:drake_cc.bzl", "drake_cc_library")
load(
    "//tools/workspace/mkdoc_internal:defs.bzl",
    "generate_pybind_documentation_header",
)

def generate_docstrings(*, subdir):
    """Given a subdir name like "multibody/tree", declares a cc_library named
    ":multibody_tree" that contains "multibody_tree.h" (committed to git), as
    well as a rule ":gen_multibody_tree" that outputs the "gen/multibody_tree.h"
    generated file. The two header files (committed vs generated) are checked
    for equality by the `:diff_test:` in our neighboring `BUILD.bazel` file.
    """
    identifier = subdir.replace("/", "_")
    filename = "{}.h".format(identifier)

    # Declare the committed docstrings.
    drake_cc_library(
        name = identifier,
        hdrs = [filename],
        declare_installed_headers = False,
        tags = ["nolint"],
        visibility = ["//bindings:__subpackages__"],
    )

    # Generate the reference docstrings.
    generate_pybind_documentation_header(
        name = "gen_{}".format(identifier),
        out = "gen/{}".format(filename),
        hdr_subdir = "drake/{}".format(subdir),
        exclude_hdr_patterns = [
            # Anonymous namespace and deduction guides both confuse pybind.
            "drake/common/overloaded.h",
            # These headers are deprecated for removal on 2026-07-01, and are
            # conditionally omitted when the use_eigen_legacy_autodiff is off.
            # To avoid confusing the docstrings diff_test, we'll omit them from
            # generation entirely, since we don't need any of their docs anyway.
            "drake/common/autodiff_overloads.h",
            "drake/common/autodiffxd.h",
            "drake/common/eigen_autodiff_types.h",
        ],
        root_name = "pydrake_doc_{}".format(identifier),
        targets = ["//tools/install/libdrake:drake_headers"],
    )
