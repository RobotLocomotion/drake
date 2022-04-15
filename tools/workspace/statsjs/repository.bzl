# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def statsjs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mrdoob/stats.js",
        commit = "5f0f917354ea2184456e5e361da5df6166306fbf",
        sha256 = "36acb189246ea209c22a0046bcf5583d053cc592f8657ee4c8269ca277798a47",  # noqa
        build_file = "@drake//tools/workspace/statsjs:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
