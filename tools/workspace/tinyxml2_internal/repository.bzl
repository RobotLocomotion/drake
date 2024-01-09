load("//tools/workspace:github.bzl", "github_archive")

def tinyxml2_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "leethomason/tinyxml2",
        commit = "10.0.0",
        sha256 = "3bdf15128ba16686e69bce256cc468e76c7b94ff2c7f391cc5ec09e40bff3839",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
