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
        commit = "v20.79.0",
        sha256 = "d255491a19c26b3f1593c686d4c07d7d2cebe1ba68d42caad87c068cfed0bf84",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/max_fallback_size.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
