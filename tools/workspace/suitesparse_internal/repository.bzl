load("@drake//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.0.1",
        sha256 = "dc2f8d5c2657c120b30cce942f634ec08fc3a4b0b10e19d3eef7790b2bec8d1e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
