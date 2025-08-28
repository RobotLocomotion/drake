load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.11.0",
        sha256 = "93ed4c4e546a49fc75884c3a8b807d5af4a91e39d191fbbc60a07380b12a35d1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
