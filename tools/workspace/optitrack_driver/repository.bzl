# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "0d71eaed42b52a549e333be3701601a2d55cfc33",
        sha256 = "52439e87ae7398bab70a3a2155c669a47de7d34d07251e61497f3fa831142d66",  # noqa
        mirrors = mirrors,
    )
