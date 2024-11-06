load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.24",
        sha256 = "227dbd12ee7b4c9742b202ed31e2c8bf96a67bc769e2863122dc749a1495bba5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
