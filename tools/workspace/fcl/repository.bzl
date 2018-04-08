# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "2b0f911841d7c116c8ac076b50b1992eb3dab22a",
        sha256 = "dc1e60ad77b36b55e68f62bdf838815f8705baad08bce9de6e322958c368a6b6",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
