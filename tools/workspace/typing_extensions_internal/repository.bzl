load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.12.1",
        sha256 = "a55560dd88cec32c7a697041cb1a6dc115d56e1fe74d2014f8f0561cffc10649",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
