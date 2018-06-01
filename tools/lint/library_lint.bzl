# -*- python -*-

load("//tools/skylark:drake_py.bzl", "py_test_isolated")

# Keep this constant in sync with library_lint_reporter.py.
_TAG_EXCLUDE_FROM_PACKAGE = "exclude_from_package"

def library_lint(
        existing_rules = None):
    """Within the current package, checks that drake_cc_package_library has been
    used correctly, reports a lint (test) error if not.  (To understand proper
    use of drake_cc_package_library, consult its API documentation.)

    Note that //examples/... packages are excluded from some checks, because
    they should generally not use drake_cc_package_library.
    """
    if existing_rules == None:
        existing_rules = native.existing_rules().values()
    package_name = "//" + native.package_name()  # e.g., "//systems/framework"
    short_package_name = package_name.split("/")[-1]  # e.g., "framework"

    # We use a python helper script to report lint errors.  As we find possible
    # lint problems, we will append arguments here that will be passed along to
    # the helper.
    library_lint_reporter_args = [
        "--package-name", package_name,
    ]
    library_lint_reporter_data = []

    # Within the current package, find all cc_library rules, and the (at most)
    # one package_library rule.
    cc_library_rules = []
    package_library_rule = None
    for one_rule in existing_rules:
        # We only want cc_library.
        if one_rule["kind"] != "cc_library":
            continue
        # Ignore magic private libraries.
        if one_rule["name"].startswith("_"):
            continue
        # Found a cc_library.
        cc_library_rules.append(one_rule)
        # Is it a package_library?
        if "drake_cc_package_library" in one_rule["tags"]:
            not package_library_rule or fail("Two package libraries?")
            package_library_rule = one_rule
        elif one_rule["name"] == short_package_name:
            # Failure to use drake_cc_package_library is a lint error, except
            # in examples folders because their libraryes are never installed.
            if not package_name.startswith("//examples"):
                library_lint_reporter_args += ["--untagged-package-library"]

    # If there is no C++ code in this package, then we're done.
    if not cc_library_rules:
        return

    # Sanity check the package_library_rule name.
    if package_library_rule and (
            package_library_rule["name"] != short_package_name):
        fail("drake_cc_package_library should not allow wrong-names?!")

    # Unless the package_library rule exists and is testonly, then we should
    # exclude testonly cc_library targets from the scope we're going to insist
    # that it covers.
    exclude_testonly = not (package_library_rule or {}).get("testonly", False)

    # We are going to run genquery over all of this package's cc_library rules.
    scope = [
        package_name + ":" + one_rule["name"]
        for one_rule in cc_library_rules
    ]
    all_libraries = " + ".join(scope)

    # This expression computes the exact result for what we want the deps of
    # the drake_cc_package_library to be.
    correct_deps_expression = " ".join([
        # Start with all this package's cc_library rules.
        "({})".format(all_libraries),
        # Remove items that have opted-out of the package_library.
        "except attr(tags, '{}', {})".format(
            _TAG_EXCLUDE_FROM_PACKAGE, all_libraries),
        # Maybe remove libraries tagged testonly = 1.
        "except attr(testonly, 1, {})".format(
            all_libraries) if exclude_testonly else "",
    ])

    # Find libraries that are deps of the package_library but shouldn't be.
    extra_deps_expression = "deps({}, 1) except ({})".format(
        package_name, correct_deps_expression)
    # Find libraries that should be deps of the package_library but aren't.
    # Note that our library_lint_reporter.py tool filters out some false
    # positives from this report.
    missing_deps_expression = "({}) except deps({}, 1) ".format(
        correct_deps_expression, package_name)

    # If there was a package_library rule, ensure its deps are comprehensive.
    if package_library_rule:
        # If snopt is disabled, we have to use a dummy scope and expression.
        #
        # Details: When snopt is enabled, we obtain the @snopt source code via
        # git.  When snopt is disabled, there is no guarantee that git is still
        # going to work (e.g., the repository is authenticated).  Normally this
        # is not a problem.  However, during a `bazel query` or `genquery()`
        # computation, Bazel still builds a graph where the conditional
        # dependency on snopt is reified, and so its repository_rule gets run
        # (and fails).  Thus, we shouldn't do any genquery() operations unless
        # snopt is enabled.  Instead, we'll scope the genquery to a dummy
        # label, and nerf the expression to be definitionally empty.
        #
        # TODO(jwnimmer-tri) Figure out how make linting work even when snopt
        # is disabled.
        dummy = "@bazel_tools//tools/cpp:empty"
        dummy_expression = "{} except {}".format(dummy, dummy)
        native.genquery(
            name = "library_lint_missing_deps",
            expression = select({
                "//tools:with_snopt": missing_deps_expression,
                "//conditions:default": dummy_expression,
            }),
            scope = select({
                "//tools:with_snopt": scope,
                "//conditions:default": [dummy],
            }),
            testonly = 1,
            visibility = ["//visibility:private"],
        )
        native.genquery(
            name = "library_lint_extra_deps",
            expression = select({
                "//tools:with_snopt": extra_deps_expression,
                "//conditions:default": dummy_expression,
            }),
            scope = select({
                "//tools:with_snopt": scope,
                "//conditions:default": [dummy],
            }),
            testonly = 1,
            visibility = ["//visibility:private"],
        )
        library_lint_reporter_data += [
            ":library_lint_missing_deps",
            ":library_lint_extra_deps",
        ]
        library_lint_reporter_args += [
            "--missing-deps", "$(location :library_lint_missing_deps)",
            "--extra-deps", "$(location :library_lint_extra_deps)",
        ]

    # Report all of the library_lint results.
    py_test_isolated(
        name = "library_lint",
        srcs = ["@drake//tools/lint:library_lint_reporter.py"],
        main = "@drake//tools/lint:library_lint_reporter.py",
        args = library_lint_reporter_args,
        data = library_lint_reporter_data,
        tags = ["lint", "library_lint"],
    )
