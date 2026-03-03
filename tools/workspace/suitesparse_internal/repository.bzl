load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.12.2",
        sha256 = "679412daa5f69af96d6976595c1ac64f252287a56e98cc4a8155d09cc7fd69e8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
