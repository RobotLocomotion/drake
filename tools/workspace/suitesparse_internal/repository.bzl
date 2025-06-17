load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.10.3",
        sha256 = "09e7bcc8e5de0a5b55d2ae9fd6378d5f4dc1b85a933593339a0872b24e2cc102",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
