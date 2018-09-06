# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "91d9d4d5735e44f91ba9df013c9fcd16a22938e4",
        sha256 = "bf878bcd92f97f5eea697896812014985bad3c98032abf8e72123b673db2cbeb",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
