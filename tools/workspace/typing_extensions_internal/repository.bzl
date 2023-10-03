load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.8.0",
        sha256 = "7c42f535c1195033230d3aa02fb772dba928058883395e7f6f791203ff9c9226",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
