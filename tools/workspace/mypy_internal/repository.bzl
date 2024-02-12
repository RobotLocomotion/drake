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
        commit = "v1.8.0",
        sha256 = "9992b74ec31aa4031baac6bc212799cf47c786005d235bda0b44efacbb89b0c7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_retry.patch",
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
