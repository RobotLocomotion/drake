# -*- mode: python -*-
# vi: set ft=python :

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _bazel_lint(name, files, ignore):
    if files:
        if ignore:
            ignore = ["--ignore=" + ",".join(["E%s" % e for e in ignore])]

        native.py_test(
            name = name + "_codestyle",
            size = "small",
            srcs = ["@drake//tools:bzlcodestyle"],
            data = files,
            args = ignore + ["$(location %s)" % f for f in files],
            main = "@drake//tools:bzlcodestyle.py",
            srcs_version = "PY2AND3",
            tags = ["bzlcodestyle", "lint"],
        )

        native.sh_test(
            name = name + "_buildifier",
            size = "small",
            srcs = ["@drake//tools:buildifier-test.sh"],
            data = files + [
                "@drake//tools:buildifier",
                "@drake//tools:buildifier-tables.json",
            ],
            args = ["$(location %s)" % f for f in files],
            tags = ["buildifier", "lint"],
        )

#------------------------------------------------------------------------------
def bazel_lint(name = "bazel", ignore = [265, 302, 305]):
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
            load("//tools:bazel_lint.bzl", "bazel_lint")

            bazel_lint()
    """

    _bazel_lint(
        name = name,
        files = native.glob([
            "*.bzl",
            "*.BUILD",
            "BUILD",
            "BUILD.bazel",
            "WORKSPACE",
        ]),
        ignore = ignore,
    )
