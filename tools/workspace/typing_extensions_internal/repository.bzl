load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.10.0",
        sha256 = "c5f99473e0b6c554af5ae93b34accaa3806523156010e2270a4c8901e3ccc9d2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
