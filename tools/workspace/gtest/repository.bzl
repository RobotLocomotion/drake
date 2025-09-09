load("//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "v1.17.0",
        sha256 = "65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            "//tools/workspace/googletest:patches/add_printers.patch",
        ],
        mirrors = mirrors,
    )
