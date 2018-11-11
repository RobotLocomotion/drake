# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcmtypes_bot2_core_repository(
        name,
        mirrors = None):
    github_archive(
        name = "lcmtypes_bot2_core",
        repository = "openhumanoids/bot_core_lcmtypes",
        commit = "9974c813bf746851067bb7b9adf86816c5039987",
        sha256 = "a68d929f9e90b4c9433b4c18cd9eebcabe03c8082639cae5b3d6838a53f0001d",  # noqa
        build_file = "@drake//tools/workspace/lcmtypes_bot2_core:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
