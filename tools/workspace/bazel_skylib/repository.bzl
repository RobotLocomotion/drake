# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.4.0",
        sha256 = "4dd05f44200db3b78f72f56ebd8b102d5bcdc17c0299955d4eb20c38c6f07cd7",  # noqa
        mirrors = mirrors,
    )
