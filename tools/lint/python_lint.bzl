# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/skylark:drake_py.bzl", "py_test_isolated")

# N.B. Copied from `DEFAULT_IGNORE` in `pycodestyle.py`.
PYTHON_LINT_IGNORE_DEFAULT = "E121,E123,E126,E226,E24,E704,W503".split(",")

# Internal helper.
def _python_lint(name_prefix, files, ignore, disallow_executable):
    ignore_types = PYTHON_LINT_IGNORE_DEFAULT + (ignore or [])
    ignore_args = ["--ignore={}".format(",".join(ignore_types))]

    # Pycodestyle.
    locations = ["$(location %s)" % f for f in files]
    py_test_isolated(
        name = name_prefix + "_pycodestyle",
        size = "small",
        srcs = ["@pycodestyle//:pycodestyle"],
        data = files,
        args = ignore_args + locations,
        main = "@pycodestyle//:pycodestyle.py",
        tags = ["pycodestyle", "lint"],
    )

    # Additional Drake lint.
    drakelint_args = []
    if disallow_executable:
        drakelint_args += ["--disallow_executable"]
    drakelint_args += locations
    py_test_isolated(
        name = name_prefix + "_drakelint",
        size = "small",
        srcs = ["@drake//tools/lint:drakelint"],
        data = files,
        args = drakelint_args,
        main = "@drake//tools/lint:drakelint.py",
        tags = ["drakelint", "lint"],
    )

def python_lint(
        existing_rules = None,
        ignore = None,
        exclude = None,
        extra_srcs = None):
    """Runs the pycodestyle PEP 8 code style checker on all Python source files
    declared in rules in a BUILD file.  Also runs the drakelint.py linter.

    Args:
        existing_rules: The value of native.existing_result().values(), in case
            it has already been computed.  When not supplied, the value will be
            internally (re-)computed.
        ignore: List of errors to ignore, in addition to
            PYTHON_LINT_IGNORE_DEFAULT (as strings, with the 'E' or 'W').
        exclude: List of labels to exclude from linting, e.g., [:foo.py].
        extra_srcs: Source files that are not discoverable via rules.

    Example:
        BUILD:
            load("//tools/lint:python_lint.bzl", "python_lint")
            load("//tools/skylark:py.bzl", "py_library")

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
                s
                for s in srcs
                if s.endswith(".py") and s not in (exclude or [])
            ]
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
                disallow_executable = True,
            )
    if extra_srcs:
        _python_lint(
            name_prefix = "extra_srcs",
            files = extra_srcs,
            ignore = ignore,
            disallow_executable = False,
        )
