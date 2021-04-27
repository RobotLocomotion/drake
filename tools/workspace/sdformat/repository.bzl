# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "sdformat11_11.0.0",
        sha256 = "34c1ac9fdbd15d67a5f4dd94a0f03ea7eb037e2d7aa80ad320ca828509d5cbf7",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
