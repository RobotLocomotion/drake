# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bullet_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bulletphysics/bullet3",
        commit = "2.87",
        sha256 = "438c151c48840fe3f902ec260d9496f8beb26dba4b17769a4a53212903935f95",  # noqa
        build_file = "@drake//tools/workspace/bullet:package.BUILD.bazel",
        mirrors = mirrors,
    )
