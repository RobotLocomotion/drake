# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "sdformat11_11.1.0",
        sha256 = "a0798154e4e5874037a81c9dfbb7642c9a6e199ad3c7c5dbbd20bf0d50e4f2a9",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
