# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "a059d86deaca39d8c355a02da01b251fb62e3b80",
        sha256 = "8bf90431eacb982cd8a7715ec566f0d94e1dbab849d0a7af1db45b384559935d",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
