# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "6d008b63b801a2c85435c4ae959c28d7538ad270",
        sha256 = "20b420f43f3c590c8efc28ace7962cb5dc9fecb2b26e139070cbeee252f3176e",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
