# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        # DNM Switch back to master.
        commit = "4cdba09fdd991dc9709face2d24f660eea444134",
        sha256 = "9570e86115de64f0e38dd0eab32e389471e2a7aa1515d292919c71e4a4ba768e",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
