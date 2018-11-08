# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        # When changing the commit of OSQP used by Drake, ideally try to keep
        # Drake's commit of QDLDL aligned with what OSQP desires, e.g.,
        # https://github.com/oxfordcontrol/osqp/tree/v0.4.1/lin_sys/direct/qdldl
        # shows that v0.4.1 of OSQP prefers v0.1.3 of QDLDL.
        commit = "v0.4.1",
        sha256 = "0429d9580a9cdcce7c4ca61a9a97ccf4f24c7cab64f8dc44b3b9df73042cbac0",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
