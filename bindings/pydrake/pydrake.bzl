# -*- python -*-

load("//tools/lint:lint.bzl", "add_lint_tests")

def add_lint_tests_pydrake(**kwargs):
    data = kwargs.pop("cpplint_data", [])
    data.append("//bindings/pydrake:CPPLINT.cfg")
    data.append("//bindings/pydrake:.clang-format")
    kwargs["cpplint_data"] = data
    add_lint_tests(
        enable_clang_format_lint = True,
        **kwargs
    )
