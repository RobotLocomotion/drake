# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v19.4.0",
        sha256 = "db26a58e0ff8e8b1bf5031c3c3f24c376b9db5f28c87f17d146dede00d7c16d7",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
