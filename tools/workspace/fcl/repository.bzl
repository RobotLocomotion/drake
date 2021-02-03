# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "0c25681f551ad814f9844b215ec27dc7533b9a5a",
        sha256 = "016ef1a934e3e82c736c9e881e1cbe98f8f26d285ea58bb3e46555c59f8bec5b",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
