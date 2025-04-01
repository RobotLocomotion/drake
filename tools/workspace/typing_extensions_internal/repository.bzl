load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.13.0",
        sha256 = "ca6daaa503b09ffdd594f655ea2fbaebf0c080c932c931d3a20019da9bea35f0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
