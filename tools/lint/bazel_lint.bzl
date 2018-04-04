# -*- mode: python -*-
# vi: set ft=python :

load("//tools/skylark:drake_py.bzl", "py_test_isolated")

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _bazel_lint(name, files, ignore):
    if files:
        if ignore:
            ignore = ["--ignore=" + ",".join(["E%s" % e for e in ignore])]

        locations = ["$(locations %s)" % f for f in files]

        py_test_isolated(
            name = name + "_codestyle",
            size = "small",
            srcs = ["@drake//tools/lint:bzlcodestyle"],
            data = files,
            args = ignore + locations,
            main = "@drake//tools/lint:bzlcodestyle.py",
            srcs_version = "PY2AND3",
            tags = ["bzlcodestyle", "lint"],
        )

        py_test_isolated(
            name = name + "_buildifier",
            size = "small",
            srcs = ["@drake//tools/lint:buildifier"],
            data = files,
            args = ["-mode=check"] + locations,
            main = "@drake//tools/lint:buildifier.py",
            tags = ["buildifier", "lint"],
        )

#------------------------------------------------------------------------------
def bazel_lint(name = "bazel", ignore = None, extra_srcs = None):
    """
    Runs the ``bzlcodestyle`` code style checker on all Bazel files in the
    current directory. The tool is based on the ``pycodestyle`` :pep:`8` code
    style checker, but always disables certain checks while adding others.

    Args:
        name: Name prefix of the test (default = "bazel").
        ignore: List of errors (as integers, without the 'E') to ignore
            (default = [265, 302, 305]).

    Example:
        BUILD:
            load("//tools/lint:bazel_lint.bzl", "bazel_lint")

            bazel_lint()
    """

    if ignore == None:
        ignore = [265, 302, 305]
    if extra_srcs == None:
        extra_srcs = []

    _bazel_lint(
        name = name,
        # This this serves as the original of "bazel_lint_files" in
        # `_patterns_map` in `//tools/external_data:expose_all_files.bzl`.
        files = native.glob([
            "*.bzl",
            "*.BUILD",
            "*.BUILD.bazel",
            "BUILD",
            "BUILD.bazel",
            "WORKSPACE",
        ]) + extra_srcs,
        ignore = ignore,
    )
