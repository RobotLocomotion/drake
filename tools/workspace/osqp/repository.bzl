# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(name):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        commit = "bdc96b409a1e1660e46dadc7f6e5ff1f37d24b89",
        sha256 = "3bbed20b58eeb9c07be3f038dcfadd512f6f1cbbf52c8a55f0fe469ecfd327f6",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
    )
