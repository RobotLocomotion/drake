# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "7fcdc7f09bedb3d9544bfce067b01298873ad906",
        sha256 = "28e7f58968e147a3ee5fbbd7aa2f79c5e8b4ec4c48a94c481b6b60e8187417a9",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
