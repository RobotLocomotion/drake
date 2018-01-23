# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcmtypes_robotlocomotion_repository(name):
    github_archive(
        name = "lcmtypes_robotlocomotion",
        repository = "RobotLocomotion/lcmtypes",
        commit = "8aea7a94d53dea01bfceba5f3cbe8e8cc9fb0244",
        sha256 = "f23a143d7865ea4f6cd9aeb2211fe36e20712a39d439cf16fea2b11685f29b61",  # noqa
        build_file = "@drake//tools/workspace/lcmtypes_robotlocomotion:package.BUILD.bazel",  # noqa
)
