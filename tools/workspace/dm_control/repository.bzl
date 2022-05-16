# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "20cef21e2554592cd7fad0bb32c169aff2fe72bc",
        sha256 = "f64a027036e777517702597e222669f135aedb6fbf2b2cd455658ee1a02e13a7",  # noqa
        build_file = "@drake//tools/workspace/dm_control:package.BUILD.bazel",
        mirrors = mirrors,
    )
