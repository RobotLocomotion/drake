load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.2.0",
        sha256 = "4b1952813ce4425a39487670d964999d06f9cb89b6e84e72aa68e8af3afc2d74",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
