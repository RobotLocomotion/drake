load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "d426697a68c3eca42df981eb30c420d1f7813cb3",
        sha256 = "f22505fe3feafade93520c1a4282a824c03cda9f7611353a486cc4caee9548c2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
