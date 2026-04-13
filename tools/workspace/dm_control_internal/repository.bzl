load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.38",
        sha256 = "23e86e28ef6ba9d2fec95103d45bd2061cfed35c8b0012b1ac5ee41b080d56c6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
