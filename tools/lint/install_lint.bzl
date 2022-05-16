# -*- python -*-

load("@drake//tools/skylark:drake_py.bzl", "py_test_isolated")

def install_lint(
        existing_rules = None):
    """Within the current package, checks that any install rules are
    depended-on by Drake's master //:install rule.
    """
    if existing_rules == None:
        existing_rules = native.existing_rules().values()
    package_name = "//" + native.package_name()  # e.g., "//systems/framework"

    # For each rule tagged as "install", find a dependency chain from
    # //:install that reaches it. When there is no such chain, it is likely
    # that the developer forgot to list their package in the install.
    query_results = []
    for one_rule in existing_rules:
        tags = one_rule.get("tags")
        if "install" not in tags:
            continue
        if "nolint" in tags:
            continue
        if "no_install_lint" in tags:
            continue

        rule_name = one_rule["name"]
        rule_label = "{}:{}".format(package_name, rule_name)
        if rule_label == "//:install":
            # Don't lint a self-loop.
            continue

        genquery_name = "install_lint_genquery_{}".format(rule_name)
        native.genquery(
            name = genquery_name,
            expression = "somepath(//:install, {})".format(rule_label),
            scope = [
                "//:install",
                rule_label,
            ],
            testonly = 1,
            tags = ["lint"],
            visibility = ["//visibility:private"],
        )
        query_results.append(genquery_name)

    # Report all of the install_lint results.
    if query_results:
        args = []
        data = []
        for label in query_results:
            args.append("--genquery_output=$(location {})".format(label))
            data.append(label)
        py_test_isolated(
            name = "install_lint",
            size = "small",
            srcs = ["@drake//tools/lint:install_lint_reporter.py"],
            main = "@drake//tools/lint:install_lint_reporter.py",
            args = args,
            data = data,
            tags = ["lint", "install_lint"],
        )
