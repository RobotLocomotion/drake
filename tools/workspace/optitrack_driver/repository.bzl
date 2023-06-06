load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "f9dab231878f612db5d7df45f1f9faf08d7a4319",
        sha256 = "6d9355995113a40dc4e8e2dbddc1f5b615e1fde40a513a7cca77aae1f2be9304",  # noqa
        mirrors = mirrors,
    )
