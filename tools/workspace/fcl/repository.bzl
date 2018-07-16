# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "07c8480f70e4b5bfa15cb37b76fc5271d2502e82",
        sha256 = "75e12f5e6d93ce7ac50b7cd815767e349f09ef2bb6fe075c35c3ad5318122741",  # noqa  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
