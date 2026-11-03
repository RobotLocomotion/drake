load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        upgrade_type = "release",
        commit = "1.0.50",
        sha256 = "27b6f88ba9ff3da88e679d5aea81a5cacf74d8a1d7c47fb252d7b2146bc0762c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
