# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "2af546bfb7aa9ce3ceb8bc9c3e288a2bc355138a",
        sha256 = "684907931eff93e3bb20c5d095b1ffc363030bf72743d00b740c8edc39c5f696",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
