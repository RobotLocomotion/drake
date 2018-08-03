# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "8fb2ce0e3463ab7e431d4e5afee05724d45c2eeb",
        sha256 = "24a81577b6b01b17755e1bd38dfd0a35439ae0c559f787a3da44052a2b0373db",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
