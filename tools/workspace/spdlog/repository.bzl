# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v0.16.3",
        sha256 = "b88d7be261d9089c817fc8cee6c000d69f349b357828e4c7f66985bc5d5360b8",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
