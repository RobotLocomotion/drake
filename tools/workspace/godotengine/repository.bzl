# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def godotengine_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "godotengine/godot",
        commit = "edc3e6b0daf4acfeb3565f0f799d304d945e5a0a",
        sha256 = "136c1e958e75de15cf1dddfdd53906cbedb02343338a986694f50e1bfd5014e5",  # noqa
        build_file = "@drake//tools/workspace/godotengine:package.BUILD.bazel",
        mirrors = mirrors,
    )
