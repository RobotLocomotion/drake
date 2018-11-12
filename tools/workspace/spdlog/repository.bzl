# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v1.1.0",
        sha256 = "3dbcbfd8c07e25f5e0d662b194d3a7772ef214358c49ada23c044c4747ce8b19",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
