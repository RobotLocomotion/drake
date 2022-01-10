# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.9.0",
        sha256 = "91897568291a2081ffc1ae27c354446cc130c168a25892b2289f25e185af8676",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
