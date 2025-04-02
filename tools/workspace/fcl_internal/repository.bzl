load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "a3fbc9fe4f619d7bb1117dc137daa497d2de454b",
        sha256 = "dd542daa13ab2f861f29792f20be50a1776859540c46360b728d1fad5d819a0c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
