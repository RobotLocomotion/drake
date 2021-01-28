# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "798901e446549fe7709fabd46fd36e105303ef23",
        sha256 = "10896acc6d11a2cba9e8b2b3f1e55e20f7a944d64429173430895bda04bb2dbd",  # noqa
        mirrors = mirrors,
    )
