load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.21",
        sha256 = "ffe86d64f5134a3290cf6fa9b3a5fcac69c01e05db2ccee152a400f03844e6c4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
