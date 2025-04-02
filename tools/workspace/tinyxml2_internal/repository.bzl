load("//tools/workspace:github.bzl", "github_archive")

def tinyxml2_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "leethomason/tinyxml2",
        commit = "11.0.0",
        sha256 = "5556deb5081fb246ee92afae73efd943c889cef0cafea92b0b82422d6a18f289",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
