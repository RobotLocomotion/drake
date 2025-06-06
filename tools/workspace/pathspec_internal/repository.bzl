load("//tools/workspace:github.bzl", "github_archive")

def pathspec_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpburnz/python-pathspec",
        commit = "v0.12.1",
        sha256 = "dd47a400b58c965c93e1ee6723b8ac562ade44ebfcc12421075ebc8dbe7030a7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
