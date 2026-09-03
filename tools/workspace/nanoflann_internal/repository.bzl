load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        upgrade_type = "release",
        commit = "1.12.1",
        sha256 = "f4884bc47cdf175700ba1437293d4fadff1b8db5d968550899c63f7144f9034b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
