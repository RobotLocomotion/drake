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
        commit = "v1.16.1",
        sha256 = "c92779a6f0215c913442f2abf1fca615b253fe6d671f8de857fc34321c269ff2",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/reject_double_colon.patch",
            ":patches/no_retry.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
