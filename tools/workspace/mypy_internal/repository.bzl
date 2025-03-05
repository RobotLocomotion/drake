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
        commit = "v1.15.0",
        sha256 = "5ccbe3a37b5823b604e4746b51e10ff4d5e98ceb2f2ef4ab56e7c2fb57bb15d9",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/reject_double_colon.patch",
            ":patches/no_retry.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
