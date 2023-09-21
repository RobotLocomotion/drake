load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v0.29.1",
        sha256 = "0bff5ff33335ffe170f9ccba544f7c4d78e7b4dd883337f2ee00f8ba3c6d85b5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
