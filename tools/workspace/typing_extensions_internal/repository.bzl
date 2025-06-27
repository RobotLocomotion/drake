load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.14.0",
        sha256 = "4b8f9a7f22c4861e6d1d0f26e2f9c584271442c8e3b0ef55130616424142a554",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
