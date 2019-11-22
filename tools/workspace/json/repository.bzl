# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def json_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.7.3",
        sha256 = "249548f4867417d66ae46b338dfe0a2805f3323e81c9e9b83c89f3adbfde6f31",  # noqa
        build_file = "@drake//tools/workspace/json:package.BUILD.bazel",
        mirrors = mirrors,
    )
