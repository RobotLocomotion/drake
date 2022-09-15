# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        # This should match the SHA referenced by uwebsockets as a git
        # submodule.
        commit = "9d3d53af5658f7a472c4fd2c0c0a837c0baa93f0",
        sha256 = "562ee841afa741144f49a982ed7ebc298928918f29c2e35957eba4896f651dc8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
