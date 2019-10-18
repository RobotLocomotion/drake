# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "08ca1a178a197c6211bacbc130dbcc09660724de",
        sha256 = "826a39c54726b8703ba60a87e803994a293961e9af1e1c24bb204688a96a3c13",  # noqa
        mirrors = mirrors,
    )
