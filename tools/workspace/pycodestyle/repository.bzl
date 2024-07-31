load("//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.12.0",
        sha256 = "c72dccf2bf7ceb603b5bd8b737a511d5241e675e90d4f75bc8a12fe81f24c094",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
