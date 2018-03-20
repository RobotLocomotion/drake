# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def json_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.1.2",
        sha256 = "e8fffa6cbdb3c15ecdff32eebf958b6c686bc188da8ad5c6489462d16f83ae54",  # noqa
        build_file = "@drake//tools/workspace/json:package.BUILD.bazel",
        mirrors = mirrors,
    )
