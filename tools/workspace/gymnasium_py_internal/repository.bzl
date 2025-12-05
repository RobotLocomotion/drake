load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.2.2",
        sha256 = "a59b1ca86bae3e6ce977d34f7f432a23fedd000476ba27f8417fba7bdedd0292",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
