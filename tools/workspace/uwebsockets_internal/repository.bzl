load("//tools/workspace:github.bzl", "github_archive")

def uwebsockets_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uWebSockets",
        commit = "v20.76.0",
        sha256 = "2a97c2dff34d15d6f6771e35c8016483d3871201ad5921167f31e7d6e2fd72e6",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/max_fallback_size.patch",
        ],
        mirrors = mirrors,
    )
