# -*- mode: python -*-
# vi: set ft=python :

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _python_lint(name, files, ignore):
    if files:
        if ignore:
            ignore = ["--ignore=" + ",".join(["E%s" % e for e in ignore])]

        native.py_test(
            name = name,
            size = "small",
            srcs = ["@pycodestyle//:pycodestyle"],
            data = files,
            args = ignore + ["$(location %s)" % f for f in files],
            main = "@pycodestyle//:pycodestyle.py",
            srcs_version = "PY2AND3",
            tags = ["pycodestyle", "lint"],
        )

#------------------------------------------------------------------------------
def python_lint(ignore = []):
    """
    Runs the pycodestyle PEP 8 code style checker on all Python source files
    declared in rules in a BUILD file.

    Args:
        ignore: List of errors (as integers, without the 'E') to ignore
            (default = []).

    Example:
        BUILD:
            load("//tools:python_lint.bzl", "python_lint")

            py_library(
                name = "foo",
                srcs = ["foo.py"],
            )

            python_lint()
    """

    for rule in native.existing_rules().values():
        srcs = rule.get("srcs", ())

        if type(srcs) == type(()):
            src_labels = list(srcs)

            _python_lint(
                name = rule["name"] + "_pycodestyle",
                files = [s for s in src_labels if s.endswith(".py")],
                ignore = ignore,
            )
