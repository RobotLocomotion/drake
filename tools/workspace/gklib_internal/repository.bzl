load("//tools/workspace:github.bzl", "github_archive")

def gklib_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "KarypisLab/GKlib",
        commit = "8bd6bad750b2b0d90800c632cf18e8ee93ad72d7",
        sha256 = "e1d59de12336731e6dde8465f05de7b907fae5d72c921cffcde217a89eaab654",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
