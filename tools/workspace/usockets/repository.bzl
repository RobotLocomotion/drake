# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        commit = "v0.7.1",
        sha256 = "1fdc5376e5ef9acf4fb673fcd5fd191da9b8d59a319e9ec7922872070a3dd21c",  # noqa
        build_file = "@drake//tools/workspace/usockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
