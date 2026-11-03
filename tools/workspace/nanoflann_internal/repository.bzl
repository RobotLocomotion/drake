load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        upgrade_type = "release",
        commit = "2.0.1",
        sha256 = "bb3991ad924248d3f20953a951c34878196c11ca112d800acc341c3808443896",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
