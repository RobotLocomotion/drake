# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.6.0",
        sha256 = "22dd3eb35c5a5042aef0bb31f718295e0b323b04f63c2443a6f11dded95d5bd9",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
