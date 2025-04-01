load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.10.1",
        sha256 = "9e2974e22dba26a3cffe269731339ae8e01365cfe921b06be6359902bd05862c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
