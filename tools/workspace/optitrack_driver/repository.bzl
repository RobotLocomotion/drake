# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "795879a7e2eb910d8e7ab88dfad54d6ef67c9599",
        sha256 = "588b380e1aafe03b1d6ddf7af242b2f5ae503abb60cd86d386caf17f045e649b",  # noqa
        mirrors = mirrors,
    )
