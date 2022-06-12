# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RussTedrake/models",
        commit = "a48aa6422741ef819cda9843e8896d02a35986e8",
        sha256 = "ef7ebcd30ba9e751ac7b1fa26c0a799e533b98e8fd3b9fd5aa422bcc8c945c57",  # noqa
        build_file = "@drake//tools/workspace/models_internal:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
