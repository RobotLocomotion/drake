load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        commit = "v7.12.1",
        sha256 = "794ae22f7e38e2ac9f5cbb673be9dd80cdaff2cdf858f5104e082694f743b0ba",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
