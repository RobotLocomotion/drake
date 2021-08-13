# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v19.3.0",
        sha256 = "6f709b4e5fe053a94a952da93c07c919b36bcb8c838c69067560ae85f97c5621",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
