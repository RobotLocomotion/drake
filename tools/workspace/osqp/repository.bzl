# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_repository(name):
    github_archive(
        name = name,
        repository = "oxfordcontrol/osqp",
        commit = "v0.2.1",
        sha256 = "d8087d8e8a4755a803263e92743e192877d53f77587db82f968c0728f609bba5",  # noqa
        build_file = "@drake//tools/workspace/osqp:package.BUILD.bazel",
    )
