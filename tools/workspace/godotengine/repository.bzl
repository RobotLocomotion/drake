# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def godotengine_repository(name):
    github_archive(
        name = name,
        repository = "godotengine/godot",
        commit = "06dd10b3905a7d48beffe0523b785d513747f511",
        sha256 = "bb305dc5cec41ef379174162c62f7056f91ba7798324a9466592c0a3420c3105",  # noqa
        build_file = "@drake//tools/workspace/godotengine:package.BUILD.bazel",
    )
