# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RussTedrake/models",
        commit = "083f4d5edc78486a1c82d705ff78213ff066ce96",
        sha256 = "d00ef56a753ed9e07b6a6288e10f8e0c16d514c0bdd0f8ade3a9d6819e415790",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
