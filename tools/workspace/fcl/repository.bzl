# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "df2702ca5e703dec98ebd725782ce13862e87fc8",
        sha256 = "3ebcf2470a3ee372440cdec4cb78d1723411c3cf84f419679c2c85317c4c2dae",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
