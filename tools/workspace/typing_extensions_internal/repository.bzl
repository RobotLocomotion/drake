load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.15.0",
        sha256 = "40e4fd945fb070e470976538741ee85add33de5e8ab2f1583e9e264d8386916b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
