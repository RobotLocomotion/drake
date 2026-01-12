load("//tools/workspace:github.bzl", "github_archive")

def tomli_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hukkin/tomli",
        commit = "2.3.0",
        sha256 = "438851e2e43c7b5c43f211bd40fefdf9e59eb733477dc48525e61de09eb33339",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
