load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.2.0",
        sha256 = "ce63c34f566d0aeae3c85fdc5b72d293f7e834d10ec0a0417b4c0823ce6c0474",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
