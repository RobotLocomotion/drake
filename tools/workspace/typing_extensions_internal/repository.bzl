load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.14.1",
        sha256 = "e600cad01cac3216bfbb433b43a8ad413f517f408613ee37aea6e1b1a291a50a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
