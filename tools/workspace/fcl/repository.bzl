# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "1b13d7b8775e6f1413fd094e12e73f15d43ac57b",
        sha256 = "024f46a5b122511a841c64fd4446638f76ea673ecbc9fc62417607ffbbd1d758",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
