# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "7a5ca928094e3dfbff9be1414b3dae67dc6fa7d8",
        sha256 = "6f6c6c7ac52794adb459c5a0c4ade06af2b12be09162a4dee4eed03197ef45fc",  # noqa
        mirrors = mirrors,
    )
