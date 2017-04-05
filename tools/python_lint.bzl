# -*- mode: python -*-
# vi: set ft=python :

"""
Runs the pycodestyle PEP 8 code style checker on all Python source files
declared in rules in a BUILD file.

Example:
    BUILD:
        load("//tools:python_lint.bzl", "python_lint")

        py_library(
            name = "foo",
            srcs = ["foo.py"],
        )

        python_lint()
"""

def python_lint():
    for rule in native.existing_rules().values():
        srcs = rule.get("srcs", ())

        if type(srcs) == type(()):
            src_labels = list(srcs)

            py_labels = [
                src_label for src_label in src_labels
                if src_label.endswith(".py")
            ]

            py_locations = [
                "$(location {})".format(py_label) for py_label in py_labels
            ]

            if len(py_locations) > 0:
                native.py_test(
                    name = rule["name"] + "_pycodestyle",
                    size = "small",
                    srcs = ["@pycodestyle//:pycodestyle"],
                    data = py_labels,
                    args = py_locations,
                    main = "@pycodestyle//:pycodestyle.py",
                    srcs_version = "PY2AND3",
                    tags = ["pycodestyle"],
                )
