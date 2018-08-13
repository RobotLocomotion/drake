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
        # https://github.com/oxfordcontrol/osqp/tree/v0.4.0/lin_sys/direct/qdldl
        # shows that v0.4.0 of OSQP prefers v0.1.2 of QDLDL.
        commit = "v0.4.0",
        sha256 = "aaf3c5553590b10eafd8214e681a218f128fb88cf5ae84b78d8b3a571dc868aa",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
        mirrors = mirrors,
    )
