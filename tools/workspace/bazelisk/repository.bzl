load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        commit = "v1.18.0",
        sha256 = "5435bdcfbca7dc3a1f68848779c6ad77503f0299ddecdd1f8d1272b88d9588d2",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/pull494.patch",
        ],
        mirrors = mirrors,
    )
