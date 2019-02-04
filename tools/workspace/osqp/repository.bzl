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
        # https://github.com/oxfordcontrol/osqp/tree/v0.5.0/lin_sys/direct/qdldl
        # shows that v0.5.0 of OSQP prefers v0.1.3 of QDLDL.
        commit = "v0.5.0",
        sha256 = "e0932d1f7bc56dbe526bee4a81331c1694d94c570f8ac6a6cb413f38904e0f64",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
