load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.16",
        sha256 = "20e4e4881d84fcb43bd673796214c2f97e4f687a22f1fb4e1406ec04e735f9d7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
