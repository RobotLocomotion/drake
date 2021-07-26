# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/skylark:drake_py.bzl", "py_test_isolated")

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _bazel_lint(name, files, ignore):
    if files:
        ignores = ["E%s" % e for e in (ignore or [])]

        # W504 relates to linebreaks around binary operators; buildifier
        # disagrees with what pycodestyle wants to do.
        ignores.append("W504")

        ignores_as_arg = ["--ignore=" + ",".join(ignores)]
        locations = ["$(locations %s)" % f for f in files]

        py_test_isolated(
            name = name + "_codestyle",
            size = "small",
            srcs = ["@drake//tools/lint:bzlcodestyle"],
            data = files,
            args = ignores_as_arg + locations,
            main = "@drake//tools/lint:bzlcodestyle.py",
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
def bazel_lint(
        name = "bazel",
        ignore = None,
        extra_srcs = None,
        exclude = None):
    """
    Runs the ``bzlcodestyle`` code style checker on all Bazel files in the
    current directory. The tool is based on the ``pycodestyle`` :pep:`8` code
    style checker, but always disables certain checks while adding others.

    Args:
        name: Name prefix of the test (default = "bazel").
        ignore: List of errors (as integers, without the 'E') to ignore
            (default = [265, 302, 305]).
        extra_srcs: List of files to lint that would otherwise be missed by the
            default glob pattern for Bazel source code.
        exclude: List to be passed to the skylark glob function for files that
            should not be linted (e.g., vendored files).

    Example:
        BUILD:
            load("//tools/lint:bazel_lint.bzl", "bazel_lint")

            bazel_lint()
    """

    if ignore == None:
        ignore = [265, 302, 305]
    if extra_srcs == None:
        extra_srcs = []
    if exclude == None:
        exclude = []

    _bazel_lint(
        name = name,
        files = native.glob([
            "*.bzl",
            "*.BUILD",
            "*.BUILD.bazel",
            "BUILD",
            "BUILD.bazel",
            "WORKSPACE",
        ], exclude = exclude) + extra_srcs,
        ignore = ignore,
    )
