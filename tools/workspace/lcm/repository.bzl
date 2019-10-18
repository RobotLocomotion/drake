# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "c22669c32b40f49a4b5e986480167dbc7783ebd6",
        sha256 = "fd1a170bb9c73c63b49d13bb7ddd3dc0fe1316621a67aa4cadec9c33492f3c32",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
