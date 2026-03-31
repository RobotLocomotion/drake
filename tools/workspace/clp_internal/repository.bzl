load("//tools/workspace:github.bzl", "github_archive")

def clp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Clp",
        commit = "releases/1.17.11",
        sha256 = "2c078e174dc1a7a308e091b6256fb34b4017897fc140ea707ba207b2913ea46d",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/missing_include.patch",
        ],
        mirrors = mirrors,
    )
