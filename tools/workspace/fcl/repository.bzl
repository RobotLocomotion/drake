# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "a13c681e41eb8180cba7d4fd32637511f588cb82",
        sha256 = "25db20d88dbf8c7462b7bbd52db559b05b49e464970bf372dc4c3c61a92ee9eb",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
