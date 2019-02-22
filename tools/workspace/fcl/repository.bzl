# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "9082fd2be665b2245716fdcb22ff5ae6ad29d892",
        sha256 = "7cb8e7ac9358ec6ad6d246aa08cfe08a90a088d43defe4e96789dfd71f6571d6",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
