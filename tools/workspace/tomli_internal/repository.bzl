load("//tools/workspace:github.bzl", "github_archive")

def tomli_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hukkin/tomli",
        commit = "2.2.1",
        sha256 = "3af7c4b571d1ccddaba020c558da0ce5b5e24edc830e478a903d82dc2d9013ae",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
