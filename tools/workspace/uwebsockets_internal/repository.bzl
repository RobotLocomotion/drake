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
        upgrade_type = "release",
        commit = "v20.81.0",
        sha256 = "1f33f5d633a912e8c819925ca949d78723c65c66bd82a1214a273bc721a2d7ba",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/max_fallback_size.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
