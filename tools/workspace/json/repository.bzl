# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def json_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.6.1",
        sha256 = "80c45b090e40bf3d7a7f2a6e9f36206d3ff710acfa8d8cc1f8c763bb3075e22e",  # noqa
        build_file = "@drake//tools/workspace/json:package.BUILD.bazel",
        mirrors = mirrors,
    )
