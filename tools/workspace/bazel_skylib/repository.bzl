# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.1.1",
        sha256 = "07b4117379dde7ab382345c3b0f5edfc6b7cff6c93756eac63da121e0bbcc5de",  # noqa
        mirrors = mirrors,
    )
