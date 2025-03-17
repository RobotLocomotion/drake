load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.46",
        sha256 = "767b01936f9620d4ab4cdf6ec348f6526f861f825648b610b1d604167dc738d2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
