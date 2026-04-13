load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.56",
        sha256 = "41d74ffe235cb7e8bab40bcad2167f7bb25edbf2231dcfff57ccf4305dc0bfae",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
