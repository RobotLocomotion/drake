# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "e7ab2d568510f7ec3018e1e0224c56654a4a42ef",
        sha256 = "d19f1d4f1a42017fa6d1b1c34d30f94ec93d4afa4221f860fe10a5f3b0bfeddd",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
