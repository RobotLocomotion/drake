load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.22",
        sha256 = "03a6e9f1ffe39f00b24bafd35d6f9491b268ba2c38bda98f757532350e0f70be",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
