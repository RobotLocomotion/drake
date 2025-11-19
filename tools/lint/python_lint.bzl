load("//tools/skylark:drake_py.bzl", "py_test_isolated")
load("//tools/skylark:sh.bzl", "sh_test")

# Internal helper.
def _python_lint(
        *,
        name_prefix,
        files,
        disallow_executable):
    locations = ["$(location %s)" % f for f in files]

    # Ruff lint.
    sh_test(
        name = name_prefix + "_ruff_check_lint",
        size = "small",
        srcs = ["@ruff"],
        data = ["//:.ruff.toml"] + files,
        args = ["check"] + locations,
        tags = ["ruff", "lint"],
    )

    # Ruff format.
    sh_test(
        name = name_prefix + "_ruff_format_lint",
        size = "small",
        srcs = ["//tools/lint:ruff_format_lint.sh"],
        data = ["@ruff", "//:.ruff.toml"] + files,
        args = locations,
        tags = ["ruff", "lint"],
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
        exclude = None,
        extra_srcs = None):
    """Runs the linters on all Python source files declared in rules in a BUILD
    file: ruff check, ruff format --check, and drakelint. (At the moment, the
    formatter can be opted-out but we're working to remove that option.)

    Args:
        existing_rules: The value of native.existing_result().values(), in case
            it has already been computed.  When not supplied, the value will be
            internally (re-)computed.
        exclude: List of labels to exclude from linting, e.g., [:foo.py].
        extra_srcs: Source files that are not discoverable via rules.
    """
    if existing_rules == None:
        existing_rules = native.existing_rules().values()

    # Collect all source files and de-duplicate.
    files = []
    for rule in existing_rules:
        # Disable linting when requested (e.g., for generated code).
        if "nolint" in rule.get("tags"):
            continue

        # Extract the list of python sources.
        srcs = rule.get("srcs", ())
        if type(srcs) == type(()):
            files.extend([
                s
                for s in srcs
                if s.endswith(".py") and s not in (exclude or [])
            ])
        else:
            # The select() syntax returns an object we (apparently) can't
            # inspect.  TODO(jwnimmer-tri) Figure out how to lint these files.
            pass
    files = depset(files).to_list()

    # Our `drake_py_unittest` adds this to `srcs` for all tests. We don't want
    # to lint it at those points of use, rather only in //common/test_utilities
    # where it doesn't have the full bazel package name in front of it.
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if helper in files:
        files.remove(helper)

    # Add a lint test if necessary.
    if len(files) > 0:
        _python_lint(
            name_prefix = "package",
            files = files,
            disallow_executable = True,
        )

    # We must lint these separately due to `disallow_executable` varying.
    if extra_srcs:
        _python_lint(
            name_prefix = "extra_srcs",
            files = extra_srcs,
            disallow_executable = False,
        )
