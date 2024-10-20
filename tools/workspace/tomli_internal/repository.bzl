load("//tools/workspace:github.bzl", "github_archive")

def tomli_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hukkin/tomli",
        commit = "2.0.2",
        sha256 = "a3a652f16bf326ba763ada67169165daf87ff9c465e21ad8264f2657beaf5264",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
