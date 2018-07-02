# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "9dba579158109c0164bfe0e8b4a75c16bfc788f6",
        sha256 = "52793f2c2d309aea559c9c322ee90638960610450ead7c88cd3b77e5bb97225f",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
