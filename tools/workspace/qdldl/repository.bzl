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
        # https://github.com/oxfordcontrol/osqp/tree/v0.4.1/lin_sys/direct/qdldl
        # shows that v0.4.1 of OSQP prefers v0.1.3 of QDLDL.
        commit = "v0.1.3",
        sha256 = "a2c3a7d0c6a48b2fab7400fa8ca72a34fb1e3a19964b281c73564178f97afe54",  # noqa
        build_file = "@drake//tools/workspace/qdldl:package.BUILD.bazel",
        mirrors = mirrors,
    )
