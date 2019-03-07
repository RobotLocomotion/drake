# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v1.3.1",
        sha256 = "160845266e94db1d4922ef755637f6901266731c4cb3b30b45bf41efa0e6ab70",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
