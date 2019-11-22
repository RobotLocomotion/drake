# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "cb60d9eea9bf1fef6376ca8f190f793b52049bc9",
        sha256 = "3c6a23ce1ed0269728b8d68f89a0994f431787235b260548d29af621bbdabf25",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
