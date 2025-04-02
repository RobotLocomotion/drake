load("//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.13.0",
        sha256 = "b1a4db0d9b8285f6643bcdb41362be6d6c94b891b13ead09c57a2513c46b717b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
