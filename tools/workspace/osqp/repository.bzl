# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.6.2",
        sha256 = "d973c33c3164caa381ed7387375347a46f7522523350a4e51989479b9d3b59c7",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
