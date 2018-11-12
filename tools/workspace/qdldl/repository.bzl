# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/qdldl",
        # When changing the commit of QDLDL used by Drake, ideally try to keep
        # it aligned with what Drake's commit of OSQP desires, e.g.,
        # https://github.com/oxfordcontrol/osqp/tree/v0.4.0/lin_sys/direct/qdldl
        # shows that v0.4.0 of OSQP prefers v0.1.2 of QDLDL.
        commit = "v0.1.2",
        sha256 = "e059b02516a5314e2a1d69c9ede910ef3a532699c76d5455a53dbff8a97f1fe5",  # noqa
        build_file = "@drake//tools/workspace/qdldl:package.BUILD.bazel",
        mirrors = mirrors,
    )
