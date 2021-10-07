# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        commit = "v0.8.1",
        sha256 = "3b33b5924a92577854e2326b3e2d393849ec00beb865a1271bf24c0f210cc1d6",  # noqa
        build_file = "@drake//tools/workspace/usockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
