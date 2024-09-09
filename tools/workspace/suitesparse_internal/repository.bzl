load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.8.2",
        sha256 = "996c48c87baaeb5fc04bd85c7e66d3651a56fe749c531c60926d75b4db5d2181",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
