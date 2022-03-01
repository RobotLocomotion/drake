# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.2.0",
        sha256 = "61352d78e4a89405853b939853cf76d7c323f90e5507f25a22fa523acb71ea14",  # noqa
        mirrors = mirrors,
    )
