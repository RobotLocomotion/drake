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
        commit = "v1.1.1",
        sha256 = "7883c878c1ed5bcc7a5ba621ac859d23be9a26baea9f2a42596dc072143b0619",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
