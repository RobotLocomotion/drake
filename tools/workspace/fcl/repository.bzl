# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "b2a749fa2ad80b24aae679c31bc8b45be8c5885c",
        sha256 = "29d6991ed9b65b0f8532c103aefdd1f5f22d5479abb1450452802e797780f6f6",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
