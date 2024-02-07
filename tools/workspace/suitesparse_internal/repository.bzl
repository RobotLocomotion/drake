load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.4.0",
        sha256 = "f9a5cc2316a967198463198f7bf10fb8c4332de6189b0e405419a7092bc921b7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
