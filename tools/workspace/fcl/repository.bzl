# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "1bddc981de578d971cc59eb54f5d248c9d803b25",
        sha256 = "568ba128b0041d74c219760fcff245e05febc254e8f6623153695b11b044e8c0",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
