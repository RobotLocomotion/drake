load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.6.0",
        sha256 = "19cbeb9964ebe439413dd66d82ace1f904adc5f25d8a823c1b48c34bd0d29ea5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
