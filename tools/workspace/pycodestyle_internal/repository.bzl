load("//tools/workspace:github.bzl", "github_archive")

def pycodestyle_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.14.0",
        sha256 = "ffcf4dc55f1e5fbdc6dd6acf5db0fd07ded534ae376eee23a742e1410b48d9ae",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
