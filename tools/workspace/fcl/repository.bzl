# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "ff8324929fdf76d6646d7b90c221a6a5bda438d8",
        sha256 = "e849040b7336a673418c962091fc2a48520b1b8cf67faedba3c67944e63a3f96",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
