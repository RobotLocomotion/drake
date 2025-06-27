load("//tools/workspace:github.bzl", "github_archive")

def uritemplate_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python-hyper/uritemplate",
        commit = "4.2.0",
        sha256 = "b75ed8dcb1446d06f5b885de7629ffd1f88f26b4f3630ace21d108084938b473",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
