# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "f15ffc21c9c0f6f6ab7d5369e8889ccbc59c2233",
        sha256 = "0581e445b1bb65198e4e9f2441e1d13249e4217433cdc0247f99ca13f67dfb25",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
