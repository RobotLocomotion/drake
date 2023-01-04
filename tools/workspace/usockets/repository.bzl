# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        commit = "v0.8.5",
        sha256 = "c52c98b7ff2c24534c17ad97d5fea8ca0cb7ff38cc933b8d08bac6e498a2ea6b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
