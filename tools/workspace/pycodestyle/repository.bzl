load("//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.12.1",
        sha256 = "231f65fbf5558e342cbad275245accb8a988d637cbeaf66508dd890f3d2d60fa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
