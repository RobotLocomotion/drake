# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcmtypes_robotlocomotion_repository(
        name,
        mirrors = None):
    github_archive(
        name = "lcmtypes_robotlocomotion",
        repository = "RobotLocomotion/lcmtypes",
        commit = "821ff4b463a9cb8f616914d87289714fcb356a92",
        sha256 = "1407f8197950a6798908bd26c2e6186cf781f7e13ce91510da2b4823f4564953",  # noqa
        build_file = "@drake//tools/workspace/lcmtypes_robotlocomotion:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
