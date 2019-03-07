# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "v1.4.0",
        sha256 = "149d7076369d261e6adbb25d713dc9e30aeba415f4fc62bb41e748b2eb229b46",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
