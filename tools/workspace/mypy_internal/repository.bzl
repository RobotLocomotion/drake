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
        commit = "v1.16.0",
        sha256 = "b2bf9faec28adae0a06df04fe5357c369c9703f8ae3bd4a8c5633c183fe90a9a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/reject_double_colon.patch",
            ":patches/no_retry.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
