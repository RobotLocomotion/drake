load("@drake//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.1.0",
        sha256 = "4cd3d161f9aa4f98ec5fa725ee5dc27bca960a3714a707a7d12b3d0abb504679",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
