load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.2.1",
        sha256 = "10bf5eb274859e187eba94a05f2e8fd318483b0551980930985bdf4ff487372f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
