# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v0.17.0",
        sha256 = "94f74fd1b3344733d1db3de2ec22e6cbeb769f93a8baa0d4a22b1f62dc7369f8",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
