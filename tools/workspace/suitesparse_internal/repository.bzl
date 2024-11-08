load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.8.3",
        sha256 = "ce39b28d4038a09c14f21e02c664401be73c0cb96a9198418d6a98a7db73a259",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
