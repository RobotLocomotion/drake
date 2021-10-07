# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.5.0",
        sha256 = "e706bf47c6e6aec37707a003d8517ae7c85d8c0e3095685f4b7a89503910ba2e",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
