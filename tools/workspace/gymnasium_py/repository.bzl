load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def gymnasium_py_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("gymnasium_py_repository")
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.2.0",
        sha256 = "4b1952813ce4425a39487670d964999d06f9cb89b6e84e72aa68e8af3afc2d74",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
