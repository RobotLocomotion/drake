# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "bcd2cf1b27979bc725608d133ddbf3adfd497365",
        sha256 = "df55e5bd99d3e8e46fe169efe18872b66d2611ac9815c4e33039ffb1ece202b3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
