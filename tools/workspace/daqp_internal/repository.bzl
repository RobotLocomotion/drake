load("//tools/workspace:github.bzl", "github_archive")

def daqp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "darnstrom/daqp",
        upgrade_type = "tag",
        commit = "v0.10.3",
        sha256 = "6105b9875100214786fce5caf2829380c3e49981757fede84e42b0008a210949",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/calloc.patch",
        ],
        mirrors = mirrors,
    )
