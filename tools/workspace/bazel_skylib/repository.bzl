# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.0.3",
        sha256 = "7ac0fa88c0c4ad6f5b9ffb5e09ef81e235492c873659e6bb99efb89d11246bcb",  # noqa
        mirrors = mirrors,
    )
