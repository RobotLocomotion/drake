# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.14.0",
        sha256 = "15cf995844a930c9a36747e8d714b94ff886b6814b5d4e3b3ee176f05681cccc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
