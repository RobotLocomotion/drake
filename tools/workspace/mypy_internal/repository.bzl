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
        commit = "v1.11.1",
        sha256 = "131f4e7667b1284e8f5de61db6a8b3afa1555c6fd4dba86c0deb12e1202b78d5",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_retry.patch",
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
