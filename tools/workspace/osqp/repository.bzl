# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        commit = "v0.3.1",
        sha256 = "6aefbec4fa4c210e14e8bb396b28e0ba089610444df830586848e124ba773cab",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
