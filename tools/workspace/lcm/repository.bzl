# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "71c24268563d862d3435eeb6d2204deb766c0dd0",
        sha256 = "61f9df86f2e1757e41fcab0f5579f39237d1b58810541063b830f77e2a3f4380",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
