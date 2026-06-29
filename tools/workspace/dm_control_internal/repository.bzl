load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        upgrade_type = "release",
        commit = "1.0.41",
        sha256 = "c1decdfa8de4628ba1246292a269ff928fe334c7391e5f14a2aeac10cb74dfa9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
