load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "330c91f41a21eacadcf8316f0a071327e3f5c017",
        sha256 = "129bde14e4ef478960a7500097134c442e0d887390326f0632c73816f89fd8a8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
