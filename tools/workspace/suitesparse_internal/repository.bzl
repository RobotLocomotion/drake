load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.9.0",
        sha256 = "bc0b3987a502913959581614ab67098f9f203a45bb424870f2342375f96dbcb7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
