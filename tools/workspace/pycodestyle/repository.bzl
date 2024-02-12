load("//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.11.1",
        sha256 = "a01fdd890c6472eebc32e8baf21e29173c35776e765c64cc83ccd09b99dc5399",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
