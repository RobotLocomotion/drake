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
        commit = "v1.1.0",
        sha256 = "5c67fc365e4b881d3858f22450c6aba60140f697196c4b07b79df5d5c54172ff",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
