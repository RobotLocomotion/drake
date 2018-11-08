# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "5a015d5d7ae9e03c64359224ae0df2e68506bcef",
        sha256 = "52457d6f33aace87b00eaf2b2046119389a3e0f5b49a4a59b335b2f4a2e85c14",  # noqa
        mirrors = mirrors,
    )
