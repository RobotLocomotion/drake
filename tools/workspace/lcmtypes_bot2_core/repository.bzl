# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcmtypes_bot2_core_repository(name):
    github_archive(
        name = "lcmtypes_bot2_core",
        repository = "openhumanoids/bot_core_lcmtypes",
        commit = "99676541398749c2aab4b5b2c38be77d268085cc",
        sha256 = "896fd3edf87c7dfaae378af12d52d233577cc495ae96b5076c48b5b9ca700b4a",  # noqa
        build_file = "@drake//tools/workspace/lcmtypes_bot2_core:package.BUILD.bazel",  # noqa
    )
