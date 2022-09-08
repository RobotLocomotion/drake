# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "f88ab0972c7dd05ff05ec3d80955717cfa9c0125",
        sha256 = "ce04a3144d36db610c5fe7c0536c468098ac23f5158f06c7cf4a578e7737a2d3",  # noqa
        mirrors = mirrors,
    )
