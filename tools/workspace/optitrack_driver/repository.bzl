load("@drake//tools/workspace:github.bzl", "github_archive")

def optitrack_driver_repository(
        name,
        mirrors = None):
    """The @optitrack_driver external is deprecated and will be removed on or
    after 2023-11-01.
    """
    github_archive(
        name = name,
        repository = "RobotLocomotion/optitrack-driver",
        commit = "8ade24ceb3f8ee10f7e0ae17d7cd55119da72873",
        sha256 = "97b7291c6921c06f351d4d802e8f1c6716e02d1de1aaf805472200ba3c1a1203",  # noqa
        mirrors = mirrors,
    )
