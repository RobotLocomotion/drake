load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.54",
        sha256 = "ba7efce137409079989df4667706c339bebfbb10e9f413474718012a13c8cd4c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
