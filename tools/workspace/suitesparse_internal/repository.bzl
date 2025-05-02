load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.10.2",
        sha256 = "98ebd840a30ddd872b38879615b6045aa800d84eae6b44efd44b6b0682507630",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
