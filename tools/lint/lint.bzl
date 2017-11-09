# -*- python -*-

load("//tools/lint:bazel_lint.bzl", "bazel_lint")
load("//tools/lint:cpplint.bzl", "cpplint")
load("//tools/lint:python_lint.bzl", "python_lint")

def add_lint_tests(
        cpplint_data = None,
        cpplint_extra_srcs = None,
        python_lint_ignore = None,
        python_lint_exclude = None,
        python_lint_extra_srcs = None,
        bazel_lint_ignore = None):
    """For every rule in the BUILD file so far, and for all Bazel files in this
    directory, adds test rules that run Drake's standard lint suite over the
    sources.  Thus, BUILD file authors should call this function at the *end*
    of every BUILD file.

    Refer to the specific linters for their semantics and argument details:
    - bazel_lint.bzl
    - cpplint.bzl
    - python_lint.bzl

    """
    existing_rules = native.existing_rules().values()
    cpplint(
        existing_rules = existing_rules,
        data = cpplint_data,
        extra_srcs = cpplint_extra_srcs)
    python_lint(
        existing_rules = existing_rules,
        ignore = python_lint_ignore,
        exclude = python_lint_exclude,
        extra_srcs = python_lint_extra_srcs)
    bazel_lint(
        ignore = bazel_lint_ignore)
