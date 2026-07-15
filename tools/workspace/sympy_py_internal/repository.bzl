load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        upgrade_advice = """
        When upgrading, check https://github.com/sympy/sympy/issues/29231 (or
        any release notes related to this issue, etc.) for a compatible upgrade
        with mpmath_py_internal.
        """,
        upgrade_type = "release",
        commit = "1.14.0",
        sha256 = "813eecbf60fdf4c692cc1cdb30b940072160f4ab0421fa5d7aaa7a8a8c596615",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
