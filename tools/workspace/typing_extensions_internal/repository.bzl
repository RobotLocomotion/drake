load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.7.0",
        sha256 = "4d8901a72bb9a2f4497fdcd7de9d351e1a90c979eba13d5edfd4c9503cae1a1c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
