load("@bazel_skylib//rules:diff_test.bzl", "diff_test")
load("//tools/lint:lint.bzl", "add_lint_tests")
load("//tools/skylark:cc.bzl", "cc_library")
load(
    "//tools/skylark:drake_cc.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_transitive_installed_hdrs_filegroup",
)
load(
    "//tools/workspace/mkdoc_internal:defs.bzl",
    "generate_pybind_documentation_header",
)

def generate_docstrings(*, subdir):
    identifier = subdir.replace("/", "_")
    filename = "{}.h".format(identifier)

    # Generate the reference docstrings.
    generate_pybind_documentation_header(
        name = "gen_{}".format(identifier),
        out = "gen/{}".format(filename),
        hdr_subdir = "drake/{}".format(subdir),
        exclude_hdr_patterns = [
            # Anonymous namespace and deduction guides both confuse pybind.
            "drake/common/overloaded.h",
        ],
        root_name = "pydrake_doc_{}".format(identifier),
        targets = ["//tools/install/libdrake:drake_headers"],
    )

    # Declare the committed docstrings.
    drake_cc_library(
        name = identifier,
        hdrs = [filename],
        declare_installed_headers = False,
        tags = ["nolint"],
        visibility = ["//bindings:__subpackages__"],
    )

    # Complain if the committed docstring differs from the reference docstring.
    diff_test(
        name = "{}_test".format(identifier),
        failure_message = "To fix, run this command:\n  cp {}/{} {}/{}".format(
            "bazel-bin/bindings/generated_docstrings/gen",
            filename,
            "bindings/generated_docstrings",
            filename,
        ),
        file1 = ":{}".format(filename),
        file2 = ":gen/{}".format(filename),
        tags = ["mkdoc"],
    )
