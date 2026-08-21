load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.55",
        sha256 = "71a2c5b1218f60c4c6d2f1954c7eb20132156cae90bdb90b566c24db002782a6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
