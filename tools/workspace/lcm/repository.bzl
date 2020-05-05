# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "3d5e2d275ecaa14106c9f4e9c842bbbce4ab738b",
        sha256 = "d783093e66105b4da0d36f6806c9cedb343cd8aa9018f946545b70e0db97296f",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
