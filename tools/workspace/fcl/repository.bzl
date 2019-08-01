# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "8cc285f9b8ba4430dad98ca4846b85586249df1f",
        sha256 = "5ff308941546b164481a5f22b96311b51ac9e7080d2f9943582cab1dba1f0c71",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
