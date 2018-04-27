# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def godotengine_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "godotengine/godot",
        commit = "9bd402698cfa299b79b9144f8d7744c308a4e085",
        sha256 = "cbec9b48d79b1fdc16a253f438fb06544a59baf71cf2c1f47c193817f2127a10",  # noqa
        build_file = "@drake//tools/workspace/godotengine:package.BUILD.bazel",
        mirrors = mirrors,
    )
