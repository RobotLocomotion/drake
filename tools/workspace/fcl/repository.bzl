# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "b611d50e1246fad8dcb445117f84259792a37e1a",
        sha256 = "daa7e04d71c4b3c32eaf20afbae6f5b80d8341fb46396e2fb503161ca903c15d",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
