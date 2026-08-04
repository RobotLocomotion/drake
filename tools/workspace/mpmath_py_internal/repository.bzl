load("@drake//tools/workspace:github.bzl", "github_archive")

def mpmath_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "mpmath/mpmath",
        upgrade_type = "release",
        upgrade_advice = """
        When upgrading, check https://github.com/sympy/sympy/issues/29231 (or
        any release notes related to this issue, etc.) for a compatible upgrade
        with sympy_internal.
        """,
        commit = "1.3.0",
        sha256 = "8f702663fa422fbbf02d15792da4b2566160df35b8a4af55fe64cba2fec2aa00",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
