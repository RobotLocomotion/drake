# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "ccc3aac8e089d8f6ccd91edecc03c6d5677ad078",
        sha256 = "c356388b8b264c16ac8cda6e99a5961e2acccd010136888372cffadabd6b28ef",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
