load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.3.0",
        sha256 = "ea290a04bd0c3dbbefe9a869a42644fc1bb61f9451040d8d88896a9df13ee620",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
