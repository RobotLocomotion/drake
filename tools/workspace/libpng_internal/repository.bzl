load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.53",
        sha256 = "b20cee717e11416d2f96ccc7d184f63730ca8cb2f03bfd0c4ed77fbc909c0bff",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
