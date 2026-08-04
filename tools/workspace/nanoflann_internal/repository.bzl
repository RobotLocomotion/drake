load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        upgrade_type = "release",
        commit = "1.10.1",
        sha256 = "9ce16ab66c9d61a529c704a913dc41947a47e29928482105cd39f3436cdb92a1",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
