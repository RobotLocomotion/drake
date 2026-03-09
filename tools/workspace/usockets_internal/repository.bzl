load("//tools/workspace:github.bzl", "github_archive")

def usockets_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uSockets",
        commit = "v0.8.8",
        sha256 = "d14d2efe1df767dbebfb8d6f5b52aa952faf66b30c822fbe464debaa0c5c0b17",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
