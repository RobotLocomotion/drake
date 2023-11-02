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
        commit = "v1.6.1",
        sha256 = "dade18fe0fb10acab09c56d618edd3e7d152710dcbfc3dd7fbece8c27b9313ba",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_retry.patch",
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
