# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# N.B. This repository is deprecated for removal on 2023-02-01.
# For details see https://github.com/RobotLocomotion/drake/pull/18156.

def picosat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal-deps/picosat",  # License: MIT
        commit = "ee542566ca89717af1b4558b0b3f226eb3b6b42d",  # v965
        sha256 = "9a047b7ba3ac1075a2288d35045585e2e3c24961f078f30ad97a313b8e539eb2",  # noqa
        build_file = "@dreal//tools:picosat.BUILD.bazel",
        mirrors = mirrors,
    )
