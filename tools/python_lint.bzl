# -*- mode: python -*-
# vi: set ft=python :

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _python_lint(name, files, ignore):
    if ignore:
        ignore = ["--ignore=" + ",".join(["E%s" % e for e in ignore])]

    native.py_test(
        name = name,
        size = "small",
        srcs = ["@pycodestyle//:pycodestyle"],
        data = files,
        args = (ignore or []) + ["$(location %s)" % f for f in files],
        main = "@pycodestyle//:pycodestyle.py",
        srcs_version = "PY2AND3",
        tags = ["pycodestyle", "lint"],
    )

#------------------------------------------------------------------------------
def python_lint(ignore = None, exclude = None):
    """
    Runs the pycodestyle PEP 8 code style checker on all Python source files
    declared in rules in a BUILD file.

    Args:
        ignore: List of errors (as integers, without the 'E') to ignore
            (default = []).
        exclude: List of labels to exclude from linting, e.g., [:foo.py].

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
        # Do not lint generated code.
        if rule.get("generator_function") in [
                "py_proto_library",
                "lcm_py_library"]:
            continue

        # Extract the list of python sources.
        srcs = rule.get("srcs", ())
        if type(srcs) == type(()):
            files = [
                s for s in srcs
                if s.endswith(".py") and s not in (exclude or [])]
        else:
            # The select() syntax returns an object we (apparently) can't
            # inspect.  TODO(jwnimmer-tri) Figure out how to lint these files.
            files = []

        # Add a lint test if necessary.
        if files:
            _python_lint(
                name = rule["name"] + "_pycodestyle",
                files = files,
                ignore = ignore,
            )
