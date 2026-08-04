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
        commit = "v20.78.0",
        sha256 = "8deea90fc34b0987dfe983af9866d52ff762358fb24c8df891a896c0035aa28e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/max_fallback_size.patch",
        ],
        mirrors = mirrors,
    )
