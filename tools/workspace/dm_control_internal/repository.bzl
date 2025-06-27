load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.31",
        sha256 = "f405f41cba31ff8b3a4c4e74d45c3fb4cce7c6aab27066a4d7546b088a1c1c56",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
