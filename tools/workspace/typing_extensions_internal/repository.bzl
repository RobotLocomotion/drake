load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.13.2",
        sha256 = "39f1d1e0a85f92bdbfa9f0315c6fbf662ba1a4d9e38b561cef97bfa3b242356a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
