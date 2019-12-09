# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "2327020490ab661bad5fb7b30d599319b8848daf",
        sha256 = "18c77a6647edf226d0b53e6b7c16d16a9f64cb68bf2228b0d6e0af89da0dd8bd",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
