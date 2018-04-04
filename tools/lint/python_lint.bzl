# -*- mode: python -*-
# vi: set ft=python :

load("//tools/skylark:drake_py.bzl", "py_test_isolated")

# Internal helper.
def _python_lint(name_prefix, files, ignore):
    if ignore:
        ignore = ["--ignore=" + ",".join(["E%s" % e for e in ignore])]

    # Pycodestyle.
    locations = ["$(location %s)" % f for f in files]
    py_test_isolated(
        name = name_prefix + "_pycodestyle",
        size = "small",
        srcs = ["@pycodestyle//:pycodestyle"],
        data = files,
        args = (ignore or []) + locations,
        main = "@pycodestyle//:pycodestyle.py",
        srcs_version = "PY2AND3",
        tags = ["pycodestyle", "lint"]
    )

    # Additional Drake lint.
    py_test_isolated(
        name = name_prefix + "_drakelint",
        size = "small",
        srcs = ["@drake//tools/lint:drakelint"],
        data = files,
        args = locations,
        main = "@drake//tools/lint:drakelint.py",
        tags = ["drakelint", "lint"]
    )

def python_lint(existing_rules = None, ignore = None, exclude = None,
                extra_srcs = None):
    """Runs the pycodestyle PEP 8 code style checker on all Python source files
    declared in rules in a BUILD file.  Also runs the drakelint.py linter.

    Args:
        existing_rules: The value of native.existing_result().values(), in case
            it has already been computed.  When not supplied, the value will be
            internally (re-)computed.
        ignore: List of errors (as integers, without the 'E') to ignore
            (default = []).
        exclude: List of labels to exclude from linting, e.g., [:foo.py].
        extra_srcs: Source files that are not discoverable via rules.

    Example:
        BUILD:
            load("//tools/lint:python_lint.bzl", "python_lint")

            py_library(
                name = "foo",
                srcs = ["foo.py"],
            )

            python_lint()
    """

    if existing_rules == None:
        existing_rules = native.existing_rules().values()
    for rule in existing_rules:
        # Disable linting when requested (e.g., for generated code).
        if "nolint" in rule.get("tags"):
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
                name_prefix = rule["name"],
                files = files,
                ignore = ignore,
            )
    if extra_srcs:
        _python_lint(
            name_prefix = "extra_srcs",
            files = extra_srcs,
            ignore = ignore,
        )
