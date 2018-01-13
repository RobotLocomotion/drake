# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stx_repository(name):
    github_archive(
        name = name,
        repository = "tcbrindle/cpp17_headers",
        commit = "e416b34c132e05487ef8d7fd197f4513d6535c1d",
        sha256 = "26a534bef07e9e0b7c9bfa4dc0ae0b154e4a5ad8d71a2487f6e33d6e3ca0dadc",  # noqa
        build_file = "@drake//tools/workspace/stx:package.BUILD.bazel",
    )
