load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.11.0",
        sha256 = "7cdd8879f526e30bfe6309256c8c0006f419131ae95fb140b1963bf7a079097c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
