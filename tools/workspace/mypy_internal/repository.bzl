load("//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy",
        commit = "v1.10.1",
        sha256 = "36c21d7e02b8c672c2181b210f500e049c9e1f6db41f77e6e5287043e24c93b4",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_retry.patch",
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
