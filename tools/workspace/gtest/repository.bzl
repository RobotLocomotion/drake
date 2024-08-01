load("//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "v1.15.2",
        sha256 = "7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/add_printers.patch",
        ],
        mirrors = mirrors,
    )
