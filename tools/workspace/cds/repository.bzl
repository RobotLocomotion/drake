# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# N.B. This repository is deprecated for removal on 2023-02-01.
# For details see https://github.com/RobotLocomotion/drake/pull/18156.

def cds_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "khizmax/libcds",  # License: BSL-1.0
        commit = "v2.3.3",
        sha256 = "f090380ecd6b63a3c2b2f0bdb27260de2ccb22486ef7f47cc1175b70c6e4e388",  # noqa
        build_file = "@dreal//tools:cds.BUILD.bazel",
        mirrors = mirrors,
    )
