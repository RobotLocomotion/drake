# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "97ac1c4e58c06f9c774ed02321cf716a21e0342d",
        sha256 = "7d8adeac7b659ec082f8e1147301be431980f4c3116d3e6b1def691fccbbf47e",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
