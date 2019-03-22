# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "5e9376248a602e9e8d0a643bb7f6be09af1c4143",
        sha256 = "63e05866a8b961cf2ddc67aeb50353f16b139a157ed3daf4fc522d0be24c8014",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
