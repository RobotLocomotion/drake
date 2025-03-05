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
        commit = "gz-utils3_3.1.1",
        sha256 = "e3cf80c00454c964d61948d13e79a790cfa14b82fbb0c76fbd8170105fb28761",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
