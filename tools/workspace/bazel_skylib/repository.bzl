# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.4.1",
        sha256 = "060426b186670beede4104095324a72bd7494d8b4e785bf0d84a612978285908",  # noqa
        mirrors = mirrors,
    )
