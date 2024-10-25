load("//tools/workspace:github.bzl", "github_archive")

def gz_utils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/gz-utils",
        commit = "gz-utils3_3.0.0",
        sha256 = "abb2b9a108c3b752091ddd5b559c0d86f061636e85f24bd547ae91d1554debbf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
