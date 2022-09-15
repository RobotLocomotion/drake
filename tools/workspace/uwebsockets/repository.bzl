# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.16.0",
        sha256 = "8847cd9f238cc71e1b0ad52beec7c55c5affc5fad4bb32c3840832a2621ecca3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
