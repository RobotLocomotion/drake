load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.34",
        sha256 = "7e5529c2608e133d3f59f8ca38b9a937f6fe7b363660dd2d4cfd5f9b63cf4670",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
