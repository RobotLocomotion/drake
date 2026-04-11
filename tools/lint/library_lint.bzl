load("//tools/skylark:drake_py.bzl", "py_test_isolated")

# Keep this constant in sync with library_lint_reporter.py.
_TAG_EXCLUDE_FROM_PACKAGE = "exclude_from_package"

def library_lint(
        existing_rules = None):
    """Within the current package, checks that drake_cc_package_library has
    been used correctly, reports a lint (test) error if not.  (To understand
    proper use of drake_cc_package_library, consult its API documentation.)

    Note that //examples/... packages are skipped, because they should not use
    drake_cc_package_library.
    """
    if existing_rules == None:
        existing_rules = native.existing_rules().values()
    if native.package_name().split("/")[0] == "examples":
        return
    package_name = "//" + native.package_name()  # e.g., "//systems/framework"
    short_package_name = package_name.split("/")[-1]  # e.g., "framework"

    # Within the current package, find all cc_library rules and the (at most)
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

        # Categorize this cc_library as a package library or not.
        if "drake_cc_package_library" in one_rule["tags"]:
            # Found a package_library.
            if package_library_rule != None:
                fail("Two package libraries?!")
            if one_rule["name"] != short_package_name:
                fail("Wrong package_library name?!")
            package_library_rule = one_rule
        elif one_rule["name"] == short_package_name:
            if "nolint" in one_rule["tags"]:
                continue
            fail("Use drake_cc_package_library when declaring a library for " +
                 "the entire directory")
        else:
            # Found a cc_library.
            cc_library_rules.append(one_rule)

    # If there is no package library, then we're done.
    if package_library_rule == None:
        return

    # Compute exactly what the deps of the drake_cc_package_library should be.
    correct_deps = []
    for cc_library in cc_library_rules:
        # Skip items that have opted-out of the package_library.
        if _TAG_EXCLUDE_FROM_PACKAGE in cc_library["tags"]:
            continue

        # Skip testonly libraries if the package is non-testonly.
        if not package_library_rule["testonly"]:
            if cc_library.get("testonly", False):
                continue

        correct_deps.append(":{}".format(cc_library["name"]))

    # Extract what the deps of the drake_cc_package_library currently are.
    current_deps = package_library_rule["deps"]

    # Compute the lint errors.
    reporter_args = ["--package-name", package_name]
    for item in correct_deps:
        if item not in current_deps:
            reporter_args += ["--missing", item]
    for item in current_deps:
        if item not in correct_deps:
            reporter_args += ["--extra", item]

    # Report all of the library_lint results.
    py_test_isolated(
        name = "library_lint",
        size = "small",
        srcs = ["@drake//tools/lint:library_lint_reporter"],
        main = "@drake//tools/lint:library_lint_reporter.py",
        args = reporter_args,
        tags = ["lint", "library_lint"],
    )
