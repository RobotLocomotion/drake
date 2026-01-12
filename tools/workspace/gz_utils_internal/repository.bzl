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
        commit = "gz-utils4_4.0.0",
        sha256 = "b06a179ea4297be8b8d09ea7a5d3d45059a3e4030c1bd256afc62f997cc992ed",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
