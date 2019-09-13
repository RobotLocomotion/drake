# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.6.0",
        sha256 = "6e00d11d1f88c1e32a4419324b7539b89e8f9cbb1c50afe69f375347c989ba2b",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
