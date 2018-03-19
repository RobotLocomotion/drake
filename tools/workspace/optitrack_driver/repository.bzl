# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "383a89ca77f4dc26de8703bba6de72e999511850",
        sha256 = "69f4c96f43dfd553f1026b5664917572ca9f6b2ca8e3f9d4f384f2cf702f3bdb",  # noqa
        mirrors = mirrors,
    )
