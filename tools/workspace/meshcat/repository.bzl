# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "7bd957ac09609d1195a4be07307cc6cdf90db3e8",
        sha256 = "64f05e353dd670b1b868317464f10a11677d5adac8759c83b042ebe52ea48b5f",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
