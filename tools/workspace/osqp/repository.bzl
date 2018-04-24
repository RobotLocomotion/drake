# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        commit = "v0.3.0",
        sha256 = "6fc0c4578f665aef8bdf1a8c8c3b474ce34f8782b100c7d06069da3435672c69",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
