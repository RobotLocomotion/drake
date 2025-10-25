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
        commit = "v1.18.2",
        sha256 = "4f0b58727dc296b92cfa3c404d31d52597de8bab0530c697f01f0d4397d6120c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/reject_double_colon.patch",
            ":patches/no_retry.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
