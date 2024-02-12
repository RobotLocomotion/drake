load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.42",
        sha256 = "fe89de292e223829859d21990d9c4d6b7e30e295a268f6a53a022611aa98bd67",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
