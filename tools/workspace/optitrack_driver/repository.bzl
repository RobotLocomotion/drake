# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "b0d633570966e08b8915dee0867747596839d06c",
        sha256 = "5f7f46273f36073dc15191fe37dc538b4b23eaeaae63de153abeaa61d1134ad6",  # noqa
    )
